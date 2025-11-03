/*
 * context.h
 *
 * PUBLIC API - Base class for pipeline data context.
 *
 * Context carries data through the pipeline and provides service injection.
 * Users should inherit from Context to add domain-specific data.
 *
 * Example implementation:
 *   class MyContext : public flowpipe::Context {
 *   public:
 *     // Data accessors
 *     void setData(std::shared_ptr<MyData> data) { data_ = data; }
 *     std::shared_ptr<MyData> getData() const { return data_; }
 *
 *   private:
 *     std::shared_ptr<MyData> data_;
 *   };
 *
 * Service injection example:
 *   // Set a service
 *   ctx.setService<TransformLookup>(transform_provider);
 *
 *   // Get a service (in stage)
 *   auto transform = ctx.getService<TransformLookup>();
 *   if (transform) {
 *     // Use the service
 *   }
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FLOWPIPE_CONTEXT_H
#define FLOWPIPE_CONTEXT_H

#include <memory>
#include <typeindex>
#include <unordered_map>

namespace flowpipe {

/**
 * @brief Base context class for pipeline processing
 *
 * Provides data container and service locator functionality for pipeline
 * stages. Derived classes should contain the actual data to be processed.
 */
class Context {
 public:
  Context() = default;
  virtual ~Context() = default;

  // Service container for dependency injection
  template <typename T>
  void setService(std::shared_ptr<T> service) {
    services_[std::type_index(typeid(T))] = service;
  }

  template <typename T>
  std::shared_ptr<T> getService() const {
    auto it = services_.find(std::type_index(typeid(T)));
    if (it != services_.end()) {
      return std::static_pointer_cast<T>(it->second);
    }
    return nullptr;
  }

  // Disable copy to prevent slicing
  Context(const Context &) = delete;
  Context &operator=(const Context &) = delete;

  // Enable move
  Context(Context &&) = default;
  Context &operator=(Context &&) = default;

 private:
  std::unordered_map<std::type_index, std::shared_ptr<void>> services_;
};

}  // namespace flowpipe

#endif  // FLOWPIPE_CONTEXT_H