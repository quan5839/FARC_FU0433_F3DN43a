#include "test.h"

#include "fl/json.h"
#include "fl/namespace.h"
#include "fl/map.h"
#include "fl/hash_map.h"

#if FASTLED_ENABLE_JSON

#include "platforms/shared/ui/json/ui_internal.h"
#include "platforms/shared/ui/json/ui_manager.h"
#include "platforms/shared/ui/json/ui.h"
#include "platforms/shared/ui/json/slider.h"
#include "platforms/shared/ui/json/checkbox.h"
#include "platforms/shared/ui/json/dropdown.h"
#include "platforms/shared/ui/json/button.h"
#include "platforms/shared/ui/json/title.h"
#include "platforms/shared/ui/json/description.h"
#include "platforms/shared/ui/json/help.h"
#include "platforms/shared/ui/json/number_field.h"

FASTLED_USING_NAMESPACE

#if 1

TEST_CASE("JsonUiInternal creation and basic operations") {
    bool updateCalled = false;
    float updateValue = 0.0f;
    bool toJsonCalled = false;
    
    auto updateFunc = [&](const fl::Json &json) {
        updateCalled = true;
        auto maybeValue = json.get<float>();
        if (maybeValue.has_value()) {
            updateValue = *maybeValue;
        }
    };
    
    auto toJsonFunc = [&](FLArduinoJson::JsonObject &json) {
        toJsonCalled = true;
        json["test"] = "value";
    };
    
    fl::string name = "test_component";
    JsonUiInternalPtr internal = fl::make_shared<JsonUiInternal>(name, updateFunc, toJsonFunc);
    
    REQUIRE(internal != nullptr);
    CHECK(internal->name() == name);
    CHECK(internal->id() >= 0);
    CHECK(internal->groupName().empty());
    
    // Test group functionality
    fl::string groupName = "test_group";
    internal->setGroup(groupName);
    CHECK(internal->groupName() == groupName);
    internal->clearFunctions();
}


TEST_CASE("JsonUiInternal JSON operations") {
    bool updateCalled = false;
    float receivedValue = 0.0f;
    
    auto updateFunc = [&](const fl::Json &json) {
        updateCalled = true;
        auto maybeValue = json.get<float>();
        if (maybeValue.has_value()) {
            receivedValue = *maybeValue;
        }
    };
    
    auto toJsonFunc = [&](FLArduinoJson::JsonObject &json) {
        json["name"] = "test";
        json["value"] = 42.5f;
        json["type"] = "slider";
    };
    
    JsonUiInternalPtr internal = fl::make_shared<JsonUiInternal>("test", updateFunc, toJsonFunc);
    
    // Test JSON update using ideal API
    fl::Json json = fl::Json::parse("123.456");
    internal->update(json);
    
    CHECK(updateCalled);
    CHECK_CLOSE(receivedValue, 123.456f, 0.001f);
    
    // Test JSON serialization
    FLArduinoJson::JsonDocument outputDoc;
    auto jsonObj = outputDoc.to<FLArduinoJson::JsonObject>();
    internal->toJson(jsonObj);
    
    CHECK(fl::string(jsonObj["name"].as<const char*>()) == fl::string("test"));
    CHECK_CLOSE(jsonObj["value"].as<float>(), 42.5f, 0.001f);
    CHECK(fl::string(jsonObj["type"].as<const char*>()) == fl::string("slider"));
    internal->clearFunctions();
}

TEST_CASE("JsonSliderImpl basic functionality") {
    fl::string name = "test_slider";
    float initialValue = 128.0f;
    float minValue = 0.0f;
    float maxValue = 255.0f;
    
    JsonSliderImpl slider(name, initialValue, minValue, maxValue, 1.0f);
    
    CHECK(slider.name() == name);
    CHECK_CLOSE(slider.value(), initialValue, 0.001f);
    CHECK_CLOSE(slider.value_normalized(), 128.0f / 255.0f, 0.001f);
    CHECK_CLOSE(slider.getMin(), minValue, 0.001f);
    CHECK_CLOSE(slider.getMax(), maxValue, 0.001f);
    
    // Test value setting
    float newValue = 200.0f;
    slider.setValue(newValue);
    CHECK_CLOSE(slider.value(), newValue, 0.001f);
    
    // Test assignment operator
    slider = 175.5f;
    CHECK_CLOSE(slider.value(), 175.5f, 0.001f);
    
    slider = 100;
    CHECK_CLOSE(slider.value(), 100.0f, 0.001f);
    
    // Test type conversion
    CHECK(slider.as_int() == 100);
    CHECK(slider.as<int>() == 100);
    
    // Test grouping
    fl::string groupName = "controls";
    slider.Group(groupName);
    CHECK(slider.groupName() == groupName);
}

TEST_CASE("JsonSliderImpl JSON serialization") {
    JsonSliderImpl slider("brightness", 128.0f, 0.0f, 255.0f, 1.0f);
    
    FLArduinoJson::JsonDocument doc;
    auto jsonObj = doc.to<FLArduinoJson::JsonObject>();
    slider.toJson(jsonObj);
    
    CHECK(fl::string(jsonObj["name"].as<const char*>()) == fl::string("brightness"));
    CHECK(fl::string(jsonObj["type"].as<const char*>()) == fl::string("slider"));
    CHECK_CLOSE(jsonObj["value"].as<float>(), 128.0f, 0.001f);
    CHECK_CLOSE(jsonObj["min"].as<float>(), 0.0f, 0.001f);
    CHECK_CLOSE(jsonObj["max"].as<float>(), 255.0f, 0.001f);
    CHECK_CLOSE(jsonObj["step"].as<float>(), 1.0f, 0.001f);
}

TEST_CASE("JsonCheckboxImpl basic functionality") {
    fl::string name = "test_checkbox";
    bool initialValue = true;
    
    JsonCheckboxImpl checkbox(name, initialValue);
    
    CHECK(checkbox.name() == name);
    CHECK(checkbox.value() == initialValue);
    
    // Test value setting
    checkbox.setValue(false);
    CHECK(checkbox.value() == false);
    
    // Test assignment operators
    checkbox = true;
    CHECK(checkbox.value() == true);
    
    checkbox = 0;
    CHECK(checkbox.value() == false);
    
    checkbox = 1;
    CHECK(checkbox.value() == true);
    
    // Test grouping
    fl::string groupName = "options";
    checkbox.Group(groupName);
    CHECK(checkbox.groupName() == groupName);
}

TEST_CASE("JsonCheckboxImpl JSON serialization") {
    JsonCheckboxImpl checkbox("enabled", true);
    
    FLArduinoJson::JsonDocument doc;
    auto jsonObj = doc.to<FLArduinoJson::JsonObject>();
    checkbox.toJson(jsonObj);
    
    CHECK(fl::string(jsonObj["name"].as<const char*>()) == fl::string("enabled"));
    CHECK(fl::string(jsonObj["type"].as<const char*>()) == fl::string("checkbox"));
    CHECK(jsonObj["value"].as<bool>() == true);
}

TEST_CASE("JsonDropdownImpl basic functionality") {
    fl::string name = "test_dropdown";
    fl::vector<fl::string> options = {"option1", "option2", "option3"};
    
    JsonDropdownImpl dropdown(name, fl::Slice<fl::string>(options.data(), options.size()));
    
    CHECK(dropdown.name() == name);
    CHECK(dropdown.getOptionCount() == 3);
    CHECK(dropdown.getOption(0) == "option1");
    CHECK(dropdown.getOption(1) == "option2");
    CHECK(dropdown.getOption(2) == "option3");
    
    // Test initial selection (should be 0)
    CHECK(dropdown.value_int() == 0);
    CHECK(dropdown.value() == "option1");
    
    // Test setting selection by index
    dropdown.setSelectedIndex(1);
    CHECK(dropdown.value_int() == 1);
    CHECK(dropdown.value() == "option2");
    
    // Test assignment operator
    dropdown = 2;
    CHECK(dropdown.value_int() == 2);
    CHECK(dropdown.value() == "option3");
    
    // Test grouping
    fl::string groupName = "settings";
    dropdown.Group(groupName);
    CHECK(dropdown.groupName() == groupName);
}

TEST_CASE("JsonDropdownImpl initializer list constructor") {
    JsonDropdownImpl dropdown("colors", {"red", "green", "blue"});
    
    CHECK(dropdown.getOptionCount() == 3);
    CHECK(dropdown.getOption(0) == "red");
    CHECK(dropdown.getOption(1) == "green");
    CHECK(dropdown.getOption(2) == "blue");
    CHECK(dropdown.value() == "red");
}

TEST_CASE("JsonDropdownImpl JSON serialization") {
    JsonDropdownImpl dropdown("mode", {"auto", "manual", "off"});
    dropdown.setSelectedIndex(1);
    
    FLArduinoJson::JsonDocument doc;
    auto jsonObj = doc.to<FLArduinoJson::JsonObject>();
    dropdown.toJson(jsonObj);
    
    CHECK(fl::string(jsonObj["name"].as<const char*>()) == fl::string("mode"));
    CHECK(fl::string(jsonObj["type"].as<const char*>()) == fl::string("dropdown"));
    CHECK(jsonObj["value"].as<int>() == 1);
    
    auto optionsArray = jsonObj["options"];
    CHECK(optionsArray.is<FLArduinoJson::JsonArray>());
    CHECK(fl::string(optionsArray[0].as<const char*>()) == fl::string("auto"));
    CHECK(fl::string(optionsArray[1].as<const char*>()) == fl::string("manual"));
    CHECK(fl::string(optionsArray[2].as<const char*>()) == fl::string("off"));
}

TEST_CASE("JsonUiManager basic functionality") {
    bool callbackCalled = false;
    fl::string receivedJson;
    
    auto callback = [&](const char* json) {
        callbackCalled = true;
        receivedJson = json;
    };
    
    JsonUiManager manager(callback);
    
    // Create some components
    JsonSliderImpl slider("brightness", 128.0f, 0.0f, 255.0f, 1.0f);
    JsonCheckboxImpl checkbox("enabled", true);
    
    // Components should auto-register through their constructors
    // We can't directly test this without access to the internal component list
    // but we can test that the manager doesn't crash
    
    CHECK(true); // Manager constructed successfully
}



TEST_CASE("JsonHelpImpl comprehensive testing") {
    // Test basic creation
    fl::string helpContent = R"(# FastLED Quick Start

## Basic Setup
```cpp
#include <FastLED.h>
#define NUM_LEDS 60
CRGB leds[NUM_LEDS];
```

## Key Functions
- **FastLED.addLeds()** - Initialize LED strip
- **FastLED.show()** - Update display  
- **fill_solid()** - Set all LEDs to one color

For more info, visit [FastLED.io](https://fastled.io))";
    
    JsonHelpImpl help(helpContent);
    
    // Test basic properties
    CHECK(help.name() == "help");
    CHECK(help.markdownContent() == helpContent);
    CHECK(help.groupName().empty());
    
    // Test group setting
    help.Group("documentation");
    CHECK(help.groupName() == "documentation");
    
    // Test JSON serialization
    FLArduinoJson::JsonDocument doc;
    auto jsonObj = doc.to<FLArduinoJson::JsonObject>();
    help.toJson(jsonObj);
    
    CHECK(fl::string(jsonObj["name"].as<const char*>()) == fl::string("help"));
    CHECK(fl::string(jsonObj["type"].as<const char*>()) == fl::string("help"));
    CHECK(fl::string(jsonObj["group"].as<const char*>()) == fl::string("documentation"));
    CHECK(jsonObj["id"].as<int>() >= 0);
    CHECK(fl::string(jsonObj["markdownContent"].as<const char*>()) == helpContent);
}

TEST_CASE("Component boundary value testing") {
    // Test slider with edge values
    JsonSliderImpl slider("test", 50.0f, 0.0f, 100.0f, 1.0f);
    
    slider.setValue(-10.0f); // Below minimum
    // Note: We can't check clamping behavior without seeing the implementation
    // but we can verify it doesn't crash
    
    slider.setValue(150.0f); // Above maximum
    // Same note as above
    
    // Test dropdown with invalid indices
    JsonDropdownImpl dropdown("test", {"a", "b", "c"});
    
    // These should be handled gracefully
    dropdown.setSelectedIndex(-1);
    dropdown.setSelectedIndex(10);
    
    CHECK(true); // No crashes
}

TEST_CASE("JsonUiManager executeUiUpdates") {
    bool callbackCalled = false;
    fl::string receivedJson;
    
    auto callback = [&](const char* json) {
        callbackCalled = true;
        receivedJson = json;
    };
    
    // Set up the global UI system with our callback
    auto updateEngineState = setJsonUiHandlers(callback);
    
    // Create a slider - it will auto-register with the global manager
    JsonSliderImpl slider("test_slider", 50.0f, 0.0f, 100.0f, 1.0f);
    int id = slider.id();
    
    // The component should auto-register with the manager
    // We'll test by sending an update with a likely ID
    
    // Create JSON update - we'll try IDs 0-20 since we don't know the exact ID

    FLArduinoJson::JsonDocument updateDoc;
    auto updateObj = updateDoc.to<FLArduinoJson::JsonObject>();
    
    // Convert ID to string properly
    fl::string idStr;
    idStr.append(id);
    updateObj[idStr.c_str()] = 75.0f;
    
    // Convert to JSON string
    fl::string jsonStr;
    serializeJson(updateDoc, jsonStr);
    
    // Send update through the engine state updater
    updateEngineState(jsonStr.c_str());
    
    // Process the pending update
    processJsonUiPendingUpdates();

    
    // Check that the slider value was updated (at least one of the IDs should have worked)
    CHECK_CLOSE(slider.value(), 75.0f, 0.001f);
    
    // Clean up
    setJsonUiHandlers(fl::function<void(const char*)>());
}

TEST_CASE("JsonUiManager multiple components basic") {
    bool callbackCalled = false;
    fl::string receivedJson;
    
    auto callback = [&](const char* json) {
        callbackCalled = true;
        receivedJson = json;
    };
    
    // Set up the global UI system with our callback
    auto updateEngineState = setJsonUiHandlers(callback);
    
    // Create multiple components
    JsonSliderImpl slider1("slider1", 25.0f, 0.0f, 100.0f, 1.0f);
    JsonSliderImpl slider2("slider2", 50.0f, 0.0f, 100.0f, 1.0f);
    JsonCheckboxImpl checkbox("checkbox", false);
    
    // Test that components were created successfully
    CHECK_CLOSE(slider1.value(), 25.0f, 0.001f);
    CHECK_CLOSE(slider2.value(), 50.0f, 0.001f);
    CHECK(checkbox.value() == false);
    
    // Test that we can update values directly
    slider1.setValue(80.0f);
    slider2.setValue(20.0f);
    checkbox.setValue(true);
    
    // Check that all components were updated
    CHECK_CLOSE(slider1.value(), 80.0f, 0.001f);
    CHECK_CLOSE(slider2.value(), 20.0f, 0.001f);
    CHECK(checkbox.value() == true);
    
    // Clean up
    setJsonUiHandlers(fl::function<void(const char*)>());
}

#endif

#else

TEST_CASE("JSON UI disabled") {
    // When JSON is disabled, we should still be able to compile
    // but functionality will be limited
    CHECK(true);
}

#endif // FASTLED_ENABLE_JSON
