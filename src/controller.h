#pragma once
struct ControlInput  { float tiltX, tiltZ, rateX, rateZ; };  // deg, deg/s
struct ControlOutput { float gimbalA, gimbalB; };             // servo deflection, deg

class Controller {
public:
  virtual void reset() = 0;
  virtual ControlOutput update(const ControlInput& in, float dt) = 0;
  virtual ~Controller() = default;
};

class PidController : public Controller { /* ... */ };
class NeuralController : public Controller { /* weights from nn_weights.h */ };