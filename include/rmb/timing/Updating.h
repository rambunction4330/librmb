
#pragma once

#include <vector>

#include <units/time.h>

class Updatable {
public:
  Updatable(Updatable&&) = default;
  Updatable(const Updatable&) = delete;
  
  Updatable() : id(largestID++) {
    updatingObjects.push_back(*this);
  }

  ~Updatable() {
    updatingObjects.erase(std::find(updatingObjects.begin(), updatingObjects.end(), index));
  }

  static void updateAll(units::second_t period = 20_ms) {
    for (auto& object : updatingObjects) {
      object.update(period);
    }
  }

protected:
  virtual void update(units::second_t period = 20_ms) = 0;

private:
  const size_t id;

  static size_t largestID = 0;
  static std::vector<Updating&> updatingObjects;
}