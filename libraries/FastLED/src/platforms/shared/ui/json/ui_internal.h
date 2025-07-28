#pragma once



#include "fl/function.h"
#include "fl/json.h"
#include "fl/namespace.h"
#include "fl/memory.h"
#include "fl/str.h"
#include "fl/mutex.h"


namespace fl {

// JSON UI components migrated to shared_ptr system
class JsonUiInternal;
using JsonUiInternalPtr = fl::shared_ptr<JsonUiInternal>;

class JsonUiInternal {
  public:
    using UpdateFunction = fl::function<void(const fl::Json &)>;
    using ToJsonFunction = fl::function<void(FLArduinoJson::JsonObject &)>;

    JsonUiInternal(const fl::string &name, UpdateFunction updateFunc,
                 ToJsonFunction toJsonFunc);
    ~JsonUiInternal();

    const fl::string &name() const;
    void update(const fl::Json &json);
    void toJson(FLArduinoJson::JsonObject &json) const;
    int id() const;

    // Group functionality
    void setGroup(const fl::string &groupName);
    const fl::string &groupName() const;

    // Change tracking for polling (eliminates need for manual notifications)
    bool hasChanged() const;
    void markChanged();
    void clearChanged();

    bool clearFunctions();

  private:
    static int nextId();
    fl::string mName;
    UpdateFunction mUpdateFunc;
    ToJsonFunction mtoJsonFunc;
    int mId;
    fl::string mGroup;
    mutable fl::mutex mMutex;
    mutable bool mHasChanged = false; // Track if component has changed since last poll
};

} // namespace fl
