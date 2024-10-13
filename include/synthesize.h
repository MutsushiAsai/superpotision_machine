#ifndef __SUPERPOSITION_SYNTHESIZE_H__
#define __SUPERPOSITION_SYNTHESIZE_H__
#include <armadillo>

#include "common.h"
#include "constants.h"
#include "midi.h"

namespace spmidi = superposition::midi;

namespace superposition::synthesize {

struct PolygonalSynthesizerParams {
    double frequency = 0.0;
    double roll = 0.0;
    double n = 2.1;
    double teeth = 0.0;
    double fold_ratio = 0.0;
    double fm_ratio = 0.0;
    double fm_modulation = 0.0;
};

struct PolygonalSynthesizerResult {
    arma::vec out1;
    arma::vec out2;
};

class SynthesizeTask : public common::AbstractTask {
   public:
    SynthesizeTask(common::MessageBuffer<spmidi::ControlChange>* midi_buffer)
        : _midi_buffer(midi_buffer), _result_buffer(64){};
    virtual ~SynthesizeTask(){};

    virtual void process();

    common::MessageBuffer<PolygonalSynthesizerResult>* result_buffer() {
        return &_result_buffer;
    }

   private:
    common::MessageBuffer<spmidi::ControlChange>* _midi_buffer;
    common::MessageBuffer<PolygonalSynthesizerResult> _result_buffer;
};

}  // namespace superposition::synthesize

#endif