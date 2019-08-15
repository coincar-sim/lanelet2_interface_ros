#pragma once
#include "Interface.h"

namespace lanelet {
namespace interface {
namespace internal {
template <typename T>
class LazyInitialized {
public:
    explicit LazyInitialized(std::function<T()> initializer) : initializer_{std::move(initializer)} {
    }
    const T& get() const {
        return *getOrCreate();
    }
    T& get() {
        return *getOrCreate();
    }

private:
    std::unique_ptr<T>& getOrCreate() const {
        std::call_once(initialized_, [&]() { value_ = std::make_unique<T>(initializer_()); });
        return value_;
    }
    std::function<T()> initializer_;
    mutable std::unique_ptr<T> value_;
    mutable std::once_flag initialized_;
};
template <typename T>
using LazySharedFuture = LazyInitialized<std::shared_future<T>>;
} // namespace internal

//! @brief Implements an interface that locally loads the lanelet2 map.
//!
//! If multiple processes are launched, every process has to parse the map. However, all threads of one process share
//! the map (e.g. in a nodelet manager). All functions retrieve the data in a thread-safe way.
class LocalLaneletInterface : public LaneletRosInterface {
public:
    explicit LocalLaneletInterface(ParamReaderUPtr params = std::make_unique<CppRosparamReader>());

    std::shared_future<std::string> getFrameIdMapAsync() const override {
        return frameId_.get();
    }
    std::shared_future<LaneletMapConstPtr> getLaneletMapAsync() const override {
        return constMap_.get();
    }
    std::shared_future<LaneletMapPtr> getMutableLaneletMapAsync() const override {
        return map_.get();
    }
    std::shared_future<std::shared_ptr<Projector>> getProjectorAsync() const override {
        return projector_.get();
    }

private:
    internal::LazySharedFuture<std::string> frameId_;
    internal::LazySharedFuture<LaneletMapPtr> map_;
    internal::LazySharedFuture<LaneletMapConstPtr> constMap_;
    internal::LazySharedFuture<std::shared_ptr<Projector>> projector_;
};
} // namespace interface
} // namespace lanelet
