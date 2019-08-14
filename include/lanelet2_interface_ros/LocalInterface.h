#pragma once
#include "Interface.h"

namespace lanelet {
namespace interface {
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

//! Implements an interface that locally loads the lanelet2 map. If multiple processes are launched, every process has
//! to parse the map.
class LocalLaneletInterface : public LaneletRosInterface {
public:
    explicit LocalLaneletInterface(ParamReaderUPtr params = std::make_unique<CppRosparamReader>());

    std::shared_future<std::string> getFrameIdMapAsync() override {
        return frameId_.get();
    }
    std::shared_future<LaneletMapConstPtr> getLaneletMapAsync() override {
        return constMap_.get();
    }
    std::shared_future<LaneletMapPtr> getMutableLaneletMapAsync() override {
        return map_.get();
    }
    std::shared_future<std::shared_ptr<Projector>> getProjectorAsync() override {
        return projector_.get();
    }

private:
    LazySharedFuture<std::string> frameId_;
    LazySharedFuture<LaneletMapPtr> map_;
    LazySharedFuture<LaneletMapConstPtr> constMap_;
    LazySharedFuture<std::shared_ptr<Projector>> projector_;
};
} // namespace interface
} // namespace lanelet
