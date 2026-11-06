// Audio Haptics: ReactiveHaptics (update() on every 48 kHz frame, render() only when a haptics
// sample is due) reproduces process_audio_reactive_haptic_frame, the per-frame code it replaces in
// src/audio.cpp, bit for bit, for every setting, across reset() and mid-stream configure().

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
#include <vector>

#include "audio_reactive_haptics.h"

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::fprintf(stderr, "audio reactive haptics check failed at %d: %s\n", __LINE__, #condition); \
            std::exit(1); \
        } \
    } while (0)

namespace audio = ds5::audio;

namespace {

using Settings = audio::ReactiveHapticsSettings;
using Haptic = audio::HapticFrame;
using std::clamp;
using std::max;

// The reference tables and the numeric settings below rely on these values.
static_assert(AudioReactiveHapticsMix == 0 && AudioReactiveHapticsReplace == 1 && AudioReactiveHapticsBassDeep == 0
    && AudioReactiveHapticsBassBalanced == 1 && AudioReactiveHapticsBassPunchy == 2 && AudioReactiveHapticsBassWide == 3
    && AudioReactiveHapticsResponseSubtle == 0 && AudioReactiveHapticsResponseBalanced == 1
    && AudioReactiveHapticsResponseStrong == 2 && AudioReactiveHapticsAttackSoft == 0
    && AudioReactiveHapticsAttackBalanced == 1 && AudioReactiveHapticsAttackFast == 2
    && AudioReactiveHapticsAttackSharp == 3 && AudioReactiveHapticsReleaseTight == 0
    && AudioReactiveHapticsReleaseBalanced == 1 && AudioReactiveHapticsReleaseSmooth == 2
    && AudioReactiveHapticsReleaseLong == 3);

struct Frame { int16_t speaker_l, speaker_r; Haptic native; };

// The specification: process_audio_reactive_haptic_frame and its helpers as they were in src/audio.cpp.
// Globals are members without their audio_reactive_haptics_ prefix, the per-setting switches are
// tables, the macros are written out and the enabled/quiet-mode bypass stays with the caller.
// Every float expression and its operation order is the original's.
struct Reference {
    Settings s; // the original globals' initial values, pinned in test_settings()
    float filter_l = 0.0f, filter_r = 0.0f, env_l = 0.0f, env_r = 0.0f, gate = 0.0f, output_ramp = 0.0f;

    void reset() { filter_l = filter_r = env_l = env_r = gate = output_ramp = 0.0f; }

    // Indexed by enum value; anything else takes Balanced (1), as the switch defaults did.
    static float pick(uint8_t value, std::initializer_list<float> table) {
        const std::size_t index = value;
        return table.begin()[index < table.size() ? index : 1];
    }
    float filter_coeff() const { return pick(s.bass_focus, {0.01039f, 0.02074f, 0.03095f, 0.05123f}); }
    float focus_gain() const { return pick(s.bass_focus, {1.35f, 1.0f, 1.12f, 0.92f}); }
    float response_gain() const { return pick(s.response, {0.68f, 1.0f, 1.0f}); }
    float response_punch() const { return pick(s.response, {0.0f, 1.5f, 3.0f}); }
    float attack_coeff() const { return pick(s.attack, {0.20f, 0.40f, 0.65f, 0.90f}); } // 0.40f: ENVELOPE_ATTACK
    float release_coeff() const { return pick(s.release, {0.055f, 0.025f, 0.012f, 0.006f}); } // 0.025f: ..._RELEASE

    float envelope_punch(float envelope) const {
        const float normalized = clamp(envelope / 32768.0f, 0.0f, 1.0f);
        return 1.0f + (response_punch() * normalized);
    }
    static float abs_float(float value) { return value < 0.0f ? -value : value; }
    float follow_envelope(float current, float value) const {
        const float target = abs_float(value);
        const float rate = target > current ? attack_coeff() : release_coeff();
        return current + ((target - current) * rate);
    }
    static float soft_clip_unit(float value) {
        const float x = clamp(value, -4.0f, 4.0f);
        const float x2 = x * x;
        return clamp((x * (27.0f + x2)) / (27.0f + (9.0f * x2)), -1.0f, 1.0f);
    }
    static int16_t soft_clip_i16_from_float(float value) {
        const float normalized = value / 32768.0f;
        const float clipped = soft_clip_unit(normalized);
        return static_cast<int16_t>(
            clamp(static_cast<int32_t>(clipped * 32767.0f), static_cast<int32_t>(-32768), static_cast<int32_t>(32767)));
    }
    static int16_t mix_i16(int16_t native_sample, int16_t derived_sample) {
        return soft_clip_i16_from_float(static_cast<float>(native_sample) + static_cast<float>(derived_sample));
    }

    Haptic process(Frame const &f) {
        const float coeff = filter_coeff();
        filter_l += (static_cast<float>(f.speaker_l) - filter_l) * coeff;
        filter_r += (static_cast<float>(f.speaker_r) - filter_r) * coeff;
        env_l = follow_envelope(env_l, filter_l);
        env_r = follow_envelope(env_r, filter_r);

        const float peak = max(env_l, env_r);
        const float gate_target = peak > 98.0f ? 1.0f : 0.0f;         // GATE_THRESHOLD
        const float gate_rate = gate_target > gate ? 0.035f : 0.004f; // GATE_OPEN_RATE : GATE_CLOSE_RATE
        gate += (gate_target - gate) * gate_rate;
        output_ramp = std::min(1.0f, output_ramp + 0.004f);           // OUTPUT_RAMP_STEP

        const float gain = (static_cast<float>(s.gain_percent) / 100.0f) * focus_gain() * response_gain() * gate
            * output_ramp;
        const int16_t derived_l = soft_clip_i16_from_float(filter_l * gain * envelope_punch(env_l));
        const int16_t derived_r = soft_clip_i16_from_float(filter_r * gain * envelope_punch(env_r));
        if (s.mode == AudioReactiveHapticsReplace) {
            return {derived_l, derived_r};
        }
        return {mix_i16(f.native.left, derived_l), mix_i16(f.native.right, derived_r)};
    }
};

// SplitMix64 and integer arithmetic only, so every platform feeds the same samples.
struct Rng {
    uint64_t state;

    uint32_t next() {
        uint64_t z = (state += 0x9e3779b97f4a7c15ull);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
        return static_cast<uint32_t>((z ^ (z >> 31)) >> 32);
    }
    int32_t between(int32_t lo, int32_t hi) { // inclusive
        return static_cast<int32_t>(lo + static_cast<int64_t>(next() % (static_cast<uint64_t>(int64_t{hi} - lo) + 1u)));
    }
    int16_t noise() { return static_cast<int16_t>(between(-32768, 32767)); }
};

enum class Kind {
    Silence,
    Threshold, // 98, now and then +-1: the envelope wanders across the gate threshold
    Bass,      // loud 100-500 Hz triangles
    Square,    // full-scale int16 squares, 24 kHz down to 100 Hz
    Noise,     // full-range white noise
};

struct Segment { Kind kind; uint32_t frames; };

int16_t sample(Kind kind, uint32_t t, uint32_t period, int64_t amplitude, Rng &rng) {
    const uint32_t phase = t % period;
    switch (kind) {
        case Kind::Threshold: return static_cast<int16_t>(98 + (rng.next() % 4 == 0 ? rng.between(-1, 1) : 0));
        case Kind::Bass: {
            const int64_t triangle = std::abs(4 * amplitude * phase / period - 2 * amplitude) - amplitude;
            return static_cast<int16_t>(std::clamp<int64_t>(triangle + rng.between(-2000, 2000), -32768, 32767));
        }
        case Kind::Square: return phase < period / 2 ? int16_t{32767} : int16_t{-32768};
        case Kind::Noise: return rng.noise();
        case Kind::Silence: break;
    }
    return 0;
}

// The game's own haptics are noise in three segments of four, else silent.
std::vector<Frame> make_stream(uint64_t seed, std::vector<Segment> const &schedule) {
    Rng rng{seed};
    std::vector<Frame> frames;
    for (Segment const &segment : schedule) {
        const uint32_t period = static_cast<uint32_t>(rng.between(segment.kind == Kind::Square ? 2 : 96, 480));
        const int64_t amplitude_l = rng.between(2000, 32767);
        const int64_t amplitude_r = rng.between(2000, 32767);
        const uint32_t offset_r = rng.next() % period;
        const bool native = rng.next() % 4 != 0;
        for (uint32_t t = 0; t < segment.frames; ++t) {
            frames.push_back({sample(segment.kind, t, period, amplitude_l, rng),
                sample(segment.kind, t + offset_r, period, amplitude_r, rng),
                native ? Haptic{rng.noise(), rng.noise()} : Haptic{0, 0}});
        }
    }
    return frames;
}

const std::vector<Segment> kFullSchedule = {
    {Kind::Silence, 50},     // rest; the output ramp starts
    {Kind::Bass, 400},       // the gate opens while the ramp still rises
    {Kind::Threshold, 1000}, // release; the gate closes from open
    {Kind::Square, 400},
    {Kind::Silence, 800},    // decay
    {Kind::Noise, 400},
    {Kind::Bass, 550},
};

Settings make_settings(int mode, int gain_percent, int bass_focus, int response, int attack, int release) {
    using u8 = uint8_t;
    return {.mode = u8(mode), .gain_percent = uint16_t(gain_percent), .bass_focus = u8(bass_focus),
        .response = u8(response), .attack = u8(attack), .release = u8(release)};
}

// Tuning i of the 384: mode (2) x bass focus (4) x response (3) x attack (4) x release (4).
Settings tuning(int i, int gain_percent) {
    return make_settings(i / 192, gain_percent, i / 48 % 4, i / 16 % 3, i / 4 % 4, i % 4);
}

// Prints the failing case before CHECK stops the run.
bool same(Haptic got, Haptic want, Settings const &s, std::size_t frame) {
    if (got.left == want.left && got.right == want.right) {
        return true;
    }
    std::fprintf(stderr, "mode %u gain %u bass %u response %u attack %u release %u, frame %zu: got %d/%d, want %d/%d\n",
        unsigned{s.mode}, unsigned{s.gain_percent}, unsigned{s.bass_focus}, unsigned{s.response}, unsigned{s.attack},
        unsigned{s.release}, frame, got.left, got.right, want.left, want.right);
    return false;
}

// The defaults are the original globals' initial values, settings() returns
// what configure() stored, and operator== sees every field.
void test_settings() {
    Settings const defaults{};
    CHECK(defaults == make_settings(AudioReactiveHapticsMix, 100, 1, 1, 1, 1)); // Balanced tuning
    audio::ReactiveHaptics module;
    CHECK(module.settings() == defaults);
    for (int i = 0; i < 384; ++i) {
        for (int gain : {0, 1, 137, 200}) {
            module.configure(tuning(i, gain));
            CHECK(module.settings() == tuning(i, gain));
        }
    }
    CHECK(defaults == Settings{} && !(defaults != Settings{}));
    for (Settings const &other : {make_settings(1, 100, 1, 1, 1, 1), make_settings(0, 137, 1, 1, 1, 1),
             make_settings(0, 356, 1, 1, 1, 1), // gain_percent's high byte alone
             make_settings(0, 100, 0, 1, 1, 1), make_settings(0, 100, 1, 2, 1, 1),
             make_settings(0, 100, 1, 1, 3, 1), make_settings(0, 100, 1, 1, 1, 3)}) {
        CHECK(!(other == defaults) && !(defaults == other) && other != defaults);
    }
}

// One stream through the reference and the module, rendering the module (twice: render() must
// leave the state alone) on frames where frame % every == phase only.
void check_stream(Settings const &settings, uint64_t seed, uint32_t every, uint32_t phase) {
    Reference reference{settings};
    audio::ReactiveHaptics module;
    if (!(settings == Settings{})) {
        module.configure(settings); // the defaults run on a module that was never configured
    }
    const std::vector<Frame> stream = make_stream(seed, kFullSchedule);
    for (std::size_t i = 0; i < stream.size(); ++i) {
        const Haptic want = reference.process(stream[i]);
        module.update(stream[i].speaker_l, stream[i].speaker_r);
        if (i % every == phase) {
            CHECK(same(module.render(stream[i].native), want, settings, i));
            CHECK(same(module.render(stream[i].native), want, settings, i));
        }
    }
}

// Every tuning once, with a gain of 37, 100, 137 or 200, rendered on every frame or on every 16th
// frame only (the 48 kHz -> 3 kHz decimation) at some phase: had update() left any state behind or
// render() advanced any, the sparse runs would drift away from the reference.
void test_equivalence() {
    constexpr int kGains[] = {37, 100, 137, 200};
    for (int i = 0; i < 384; ++i) {
        const uint32_t pick = Rng{static_cast<uint64_t>(i)}.next();
        const bool sparse = (pick & 1) != 0;
        check_stream(tuning(i, kGains[pick / 2 % 4]), pick, sparse ? 16u : 1u, sparse ? pick / 8 % 16 : 0u);
    }
    check_stream(Settings{}, 400, 1, 0); // never configured
    check_stream(make_settings(0, 0, 0, 2, 3, 3), 401, 1, 0); // gain 0: the native haptics alone
    // configure() does not validate: out-of-range values get the Balanced
    // tuning and Mix, as the original switch defaults and mode test did.
    check_stream(make_settings(7, 137, 4, 3, 4, 255), 403, 1, 0);
    check_stream(make_settings(1, 137, 200, 255, 9, 4), 404, 16, 0);
}

// The gate opens above its threshold, not at it. The reference finds a stream whose envelope lands
// exactly on 98 under this build's floating-point code generation, and the module must follow that
// stream through the landing and for 400 frames after it.
void test_gate_threshold() {
    const Settings settings = make_settings(AudioReactiveHapticsReplace, 200, 1, 1, 1, 1);
    bool landed = false;
    for (uint64_t seed = 20000; !landed; ++seed) {
        CHECK(seed < 20016);
        Reference reference{settings};
        audio::ReactiveHaptics module;
        module.configure(settings);
        const std::vector<Frame> stream = make_stream(seed, {{Kind::Threshold, 40000}});
        std::size_t stop = stream.size();
        for (std::size_t i = 0; i < stop; ++i) {
            const Haptic want = reference.process(stream[i]);
            module.update(stream[i].speaker_l, stream[i].speaker_r);
            CHECK(same(module.render(stream[i].native), want, settings, i));
            if (!landed && max(reference.env_l, reference.env_r) == 98.0f) {
                landed = true;
                stop = std::min(stop, i + 400);
            }
        }
    }
}

// After loud history, reset() leaves a module indistinguishable from a fresh one, from render() before
// the next update() (zeroed state: no derived signal, Mix soft-clips the native sample alone) on.
void test_reset() {
    uint64_t seed = 10000;
    for (int index = 0; index < 384; index += 7) { // 55 tunings, every value of every field
        const Settings settings = tuning(index, 137);
        audio::ReactiveHaptics used, fresh;
        used.configure(settings);
        fresh.configure(settings);
        for (Frame const &f : make_stream(seed++, {{Kind::Bass, 800}, {Kind::Square, 200}})) {
            used.update(f.speaker_l, f.speaker_r);
        }
        used.reset();
        CHECK(used.settings() == settings);
        const Haptic native{20000, -32768};
        const Haptic zeroed = settings.mode == AudioReactiveHapticsReplace
            ? Haptic{0, 0}
            : Haptic{Reference::mix_i16(native.left, 0), Reference::mix_i16(native.right, 0)};
        CHECK(same(used.render(native), zeroed, settings, 0));
        CHECK(same(fresh.render(native), zeroed, settings, 0));
        Reference reference{settings};
        const std::vector<Frame> after =
            make_stream(seed++, {{Kind::Silence, 100}, {Kind::Bass, 600}, {Kind::Threshold, 400}});
        for (std::size_t i = 0; i < after.size(); ++i) {
            const Haptic want = reference.process(after[i]);
            used.update(after[i].speaker_l, after[i].speaker_r);
            fresh.update(after[i].speaker_l, after[i].speaker_r);
            CHECK(same(used.render(after[i].native), want, settings, i));
            CHECK(same(fresh.render(after[i].native), want, settings, i));
        }
    }
}

// configure() alone changes the tuning from the next frame on and keeps the filters, envelopes, gate
// and ramp; configure() then reset() (what the firmware does on a change) restarts from rest. Changes
// land at random frames: with reset(), without, or with the current settings again.
void test_configure_mid_stream() {
    for (uint64_t run = 0; run < 24; ++run) {
        Rng rng{500 + run};
        const std::vector<Frame> stream = make_stream(600 + run, kFullSchedule);
        const uint32_t every = run % 2 == 0 ? 1u : 16u;
        audio::ReactiveHaptics module;
        Reference reference;
        std::size_t next_change = static_cast<std::size_t>(rng.between(100, 1500));
        for (std::size_t i = 0; i < stream.size(); ++i) {
            if (i == next_change) {
                const uint32_t kind = rng.next() % 3; // 0: with reset(), 1: without, 2: unchanged settings
                if (kind != 2) {
                    const int index = static_cast<int>(rng.next() % 384);
                    reference.s = tuning(index, rng.between(0, 200));
                }
                module.configure(reference.s);
                if (kind == 0) {
                    module.reset();
                    reference.reset();
                }
                CHECK(module.settings() == reference.s);
                next_change += static_cast<std::size_t>(rng.between(100, 1500));
            }
            const Haptic want = reference.process(stream[i]);
            module.update(stream[i].speaker_l, stream[i].speaker_r);
            if (i % every == 0) {
                CHECK(same(module.render(stream[i].native), want, reference.s, i));
            }
        }
    }
}

} // namespace

int main() {
    test_settings();
    test_equivalence();
    test_gate_threshold();
    test_reset();
    test_configure_mid_stream();
    std::printf("Audio reactive haptics checks passed\n");
    return 0;
}
