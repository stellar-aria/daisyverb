#include <daisy_patch.h>

#include <algorithm>

#include "ap_demo.hpp"
#include "common.hpp"
#include "datorro_plate.hpp"
#include "mutable_rings.hpp"

constexpr size_t kBlockSize = 48;
constexpr size_t kNumChannels = 4;

using namespace daisy;


enum SystemModel {
  SYSTEM_MODEL_RINGS,
  SYSTEM_MODEL_PLATE,
  SYSTEM_MODEL_APDEMO,
  SYSTEM_MODEL_LAST,
};

daisy::DaisyPatch patch;
SystemModel model = SYSTEM_MODEL_RINGS;
std::array<float, 32768> DSY_SDRAM_BSS delay_line_buffer;

// Delay
MutableRings reverb_(delay_line_buffer);
DatorroPlate plate_(delay_line_buffer);
AllPassDemo apdemo_(delay_line_buffer);

/** Function that gets called at a regular interval by the hardware to
 *  process, and/or generate audio signals
 */
void AudioCallback(AudioHandle::InterleavingInputBuffer in, AudioHandle::InterleavingOutputBuffer out,
                   size_t blocksize) {
  const float strength = std::clamp(patch.controls[DaisyPatch::CTRL_1].Process(), 0.f, 1.f);
  const float size = std::clamp(patch.controls[DaisyPatch::CTRL_2].Process(), 0.f, 1.f);
  const float shape = std::clamp(patch.controls[DaisyPatch::CTRL_3].Process(), 0.f, 1.f);

  StereoSignal in_stereo{reinterpret_cast<const StereoSample*>(in), blocksize / 2};
  StereoBuffer out_stereo{reinterpret_cast<StereoSample*>(out), blocksize / 2};

  if (model == SYSTEM_MODEL_RINGS) {
    reverb_.set_amount(strength * 0.5f);
    reverb_.set_time(0.35f + 0.63f * size);
    reverb_.set_input_gain(0.2f);
    reverb_.set_lp(0.3f + shape * 0.6f);
    reverb_.Process(in_stereo, out_stereo);
  } else if (model == SYSTEM_MODEL_PLATE) {
    plate_.set_amount(strength * 0.5f);
    plate_.set_time(0.35f + 0.65f * size);
    plate_.set_input_gain(0.2f);
    plate_.set_lp(0.3f + shape * 0.7f);
    plate_.Process(in_stereo, out_stereo);
  } else if (model == SYSTEM_MODEL_APDEMO) {
    apdemo_.set_amount(strength);
    apdemo_.set_input_gain(0.2f);
    apdemo_.set_size(size);
    apdemo_.set_diffusion(shape);
    apdemo_.Process(in_stereo, out_stereo);
  }
}

void HandleUI(){
  patch.encoder.Debounce();
  if (patch.encoder.RisingEdge()) {
    model = static_cast<SystemModel>((model + 1) % SYSTEM_MODEL_LAST);
    delay_line_buffer.fill(0);
  }
}

int main(void) {
  patch.Init();
  const float sample_rate = patch.AudioSampleRate();

  delay_line_buffer.fill(0);

  apdemo_.Init(sample_rate);
  plate_.Init(sample_rate);
  reverb_.Init(sample_rate);

  patch.StartAdc();
  patch.SetAudioBlockSize(kBlockSize);

  patch.seed.StartAudio(AudioCallback);

  while(true) {
    patch.DisplayControls(false);
    HandleUI();
  };
}
