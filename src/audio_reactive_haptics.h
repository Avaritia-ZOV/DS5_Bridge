#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "audio.h"

// Audio Haptics: haptics derived from the game audio sent to the controller
// speaker. A one-pole low-pass keeps the bass, an envelope follower and a
// noise gate shape it, and the result is mixed into the game's own haptics or
// replaces them.
//
// The filters, envelopes and gate advance on every 48 kHz frame, but only the
// frames kept for the 3 kHz haptics stream need an output, so update() and
// render() are separate. Settings are resolved once, in configure().
namespace ds5::audio {

struct ReactiveHapticsSettings {
    uint8_t mode = AudioReactiveHapticsMix;
    uint16_t gain_percent = 100;
    uint8_t bass_focus = AudioReactiveHapticsBassBalanced;
    uint8_t response = AudioReactiveHapticsResponseBalanced;
    uint8_t attack = AudioReactiveHapticsAttackBalanced;
    uint8_t release = AudioReactiveHapticsReleaseBalanced;

    bool operator==(ReactiveHapticsSettings const &) const = default;
};

struct HapticFrame {
    int16_t left;
    int16_t right;
};

class ReactiveHaptics {
public:
    // The caller validates settings; out-of-range values get the Balanced
    // tuning and Mix. The filter, envelope, gate and ramp state is kept;
    // reset() clears it.
    void configure(ReactiveHapticsSettings const &settings) {
        settings_ = settings;
        tuning_ = tuning_for(settings);
    }

    ReactiveHapticsSettings const &settings() const {
        return settings_;
    }

    void reset() {
        filter_l_ = 0.0f;
        filter_r_ = 0.0f;
        envelope_l_ = 0.0f;
        envelope_r_ = 0.0f;
        gate_ = 0.0f;
        output_ramp_ = 0.0f;
    }

    // Advances the filters, envelopes, gate and output ramp by one frame.
    void update(int16_t speaker_l, int16_t speaker_r) {
        filter_l_ += (static_cast<float>(speaker_l) - filter_l_) * tuning_.filter_coeff;
        filter_r_ += (static_cast<float>(speaker_r) - filter_r_) * tuning_.filter_coeff;
        envelope_l_ = follow_envelope(envelope_l_, filter_l_);
        envelope_r_ = follow_envelope(envelope_r_, filter_r_);

        const float peak = std::max(envelope_l_, envelope_r_);
        const float gate_target = peak > kGateThreshold ? 1.0f : 0.0f;
        const float gate_rate = gate_target > gate_ ? kGateOpenRate : kGateCloseRate;
        gate_ += (gate_target - gate_) * gate_rate;
        output_ramp_ = std::min(1.0f, output_ramp_ + kOutputRampStep);
    }

    // Mixes the output for the frame last passed to update() into the native
    // haptics, or replaces them.
    HapticFrame render(HapticFrame native) const {
        const float gain = tuning_.gain * gate_ * output_ramp_;
        const HapticFrame derived{
            .left = soft_clip_i16(filter_l_ * gain * envelope_punch(envelope_l_)),
            .right = soft_clip_i16(filter_r_ * gain * envelope_punch(envelope_r_)),
        };
        if (tuning_.replace) {
            return derived;
        }
        return {.left = mix(native.left, derived.left), .right = mix(native.right, derived.right)};
    }

private:
    static constexpr float kGateThreshold = 98.0f;
    static constexpr float kGateOpenRate = 0.035f;
    static constexpr float kGateCloseRate = 0.004f;
    static constexpr float kOutputRampStep = 0.004f;

    struct Tuning {
        float filter_coeff;
        float attack_coeff;
        float release_coeff;
        float gain;
        float punch;
        bool replace;
    };

    static constexpr float filter_coeff(uint8_t bass_focus) {
        switch (bass_focus) {
            case AudioReactiveHapticsBassDeep:
                return 0.01039f;
            case AudioReactiveHapticsBassPunchy:
                return 0.03095f;
            case AudioReactiveHapticsBassWide:
                return 0.05123f;
            case AudioReactiveHapticsBassBalanced:
            default:
                return 0.02074f;
        }
    }

    static constexpr float focus_gain(uint8_t bass_focus) {
        switch (bass_focus) {
            case AudioReactiveHapticsBassDeep:
                return 1.35f;
            case AudioReactiveHapticsBassPunchy:
                return 1.12f;
            case AudioReactiveHapticsBassWide:
                return 0.92f;
            case AudioReactiveHapticsBassBalanced:
            default:
                return 1.0f;
        }
    }

    static constexpr float response_gain(uint8_t response) {
        return response == AudioReactiveHapticsResponseSubtle ? 0.68f : 1.0f;
    }

    static constexpr float response_punch(uint8_t response) {
        switch (response) {
            case AudioReactiveHapticsResponseSubtle:
                return 0.0f;
            case AudioReactiveHapticsResponseStrong:
                return 3.0f;
            case AudioReactiveHapticsResponseBalanced:
            default:
                return 1.5f;
        }
    }

    static constexpr float attack_coeff(uint8_t attack) {
        switch (attack) {
            case AudioReactiveHapticsAttackSoft:
                return 0.20f;
            case AudioReactiveHapticsAttackFast:
                return 0.65f;
            case AudioReactiveHapticsAttackSharp:
                return 0.90f;
            case AudioReactiveHapticsAttackBalanced:
            default:
                return 0.40f;
        }
    }

    static constexpr float release_coeff(uint8_t release) {
        switch (release) {
            case AudioReactiveHapticsReleaseTight:
                return 0.055f;
            case AudioReactiveHapticsReleaseSmooth:
                return 0.012f;
            case AudioReactiveHapticsReleaseLong:
                return 0.006f;
            case AudioReactiveHapticsReleaseBalanced:
            default:
                return 0.025f;
        }
    }

    static constexpr Tuning tuning_for(ReactiveHapticsSettings const &settings) {
        return {
            .filter_coeff = filter_coeff(settings.bass_focus),
            .attack_coeff = attack_coeff(settings.attack),
            .release_coeff = release_coeff(settings.release),
            .gain = (static_cast<float>(settings.gain_percent) / 100.0f)
                * focus_gain(settings.bass_focus)
                * response_gain(settings.response),
            .punch = response_punch(settings.response),
            .replace = settings.mode == AudioReactiveHapticsReplace,
        };
    }

    float follow_envelope(float current, float value) const {
        const float target = std::fabs(value);
        const float rate = target > current ? tuning_.attack_coeff : tuning_.release_coeff;
        return current + ((target - current) * rate);
    }

    float envelope_punch(float envelope) const {
        const float normalized = std::clamp(envelope / 32768.0f, 0.0f, 1.0f);
        return 1.0f + (tuning_.punch * normalized);
    }

    static int16_t soft_clip_i16(float value) {
        const float x = std::clamp(value / 32768.0f, -4.0f, 4.0f);
        const float x2 = x * x;
        const float clipped = std::clamp((x * (27.0f + x2)) / (27.0f + (9.0f * x2)), -1.0f, 1.0f);
        return static_cast<int16_t>(clipped * 32767.0f);
    }

    static int16_t mix(int16_t native, int16_t derived) {
        return soft_clip_i16(static_cast<float>(native) + static_cast<float>(derived));
    }

    ReactiveHapticsSettings settings_{};
    Tuning tuning_ = tuning_for(settings_);
    float filter_l_ = 0.0f;
    float filter_r_ = 0.0f;
    float envelope_l_ = 0.0f;
    float envelope_r_ = 0.0f;
    float gate_ = 0.0f;
    float output_ramp_ = 0.0f;
};

} // namespace ds5::audio
