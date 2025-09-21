/*
 * context.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_CONTEXT_H
#define PIPELINE_CORE_CONTEXT_H

namespace pipeline {

/**
 * @brief Base context class for pipeline processing
 *
 * A pure virtual base class that derived contexts inherit from.
 * Derived classes should contain the actual data to be processed.
 */
class Context {
public:
  Context() = default;
  virtual ~Context() = default;

  // Disable copy to prevent slicing
  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;

  // Enable move
  Context(Context &&) = default;
  Context &operator=(Context &&) = default;
};

} // namespace pipeline

#endif // PIPELINE_CORE_CONTEXT_H