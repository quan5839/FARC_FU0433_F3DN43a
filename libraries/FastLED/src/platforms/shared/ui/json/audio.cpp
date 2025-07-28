#include "platforms/shared/ui/json/audio.h"
#include "fl/json.h"
#include "fl/string.h"
#include "fl/thread_local.h"
#include "fl/warn.h"
#include "fl/thread_local.h"
#include "platforms/shared/ui/json/ui.h"

#if FASTLED_ENABLE_JSON


namespace fl {
namespace {
    fl::string& scratchBuffer() {
        static fl::ThreadLocal<fl::string> buffer;
        return buffer.access();
    }
}

JsonAudioImpl::JsonAudioImpl(const fl::string &name) {
    auto updateFunc = JsonUiInternal::UpdateFunction(
        [this](const fl::Json &json) {
            static_cast<JsonAudioImpl *>(this)->updateInternal(json);
        });

    auto toJsonFunc =
        JsonUiInternal::ToJsonFunction([this](FLArduinoJson::JsonObject &json) {
            static_cast<JsonAudioImpl *>(this)->toJson(json);
        });
    mInternal = fl::make_shared<JsonUiInternal>(name, fl::move(updateFunc),
                                       fl::move(toJsonFunc));
    mUpdater.init(this);
    addJsonUiComponent(fl::weak_ptr<JsonUiInternal>(mInternal));
}

JsonAudioImpl::~JsonAudioImpl() { removeJsonUiComponent(fl::weak_ptr<JsonUiInternal>(mInternal)); }

JsonAudioImpl &JsonAudioImpl::Group(const fl::string &name) {
    mInternal->setGroup(name);
    return *this;
}

const fl::string &JsonAudioImpl::name() const { return mInternal->name(); }

const fl::string &JsonAudioImpl::groupName() const {
    return mInternal->groupName();
}

void JsonAudioImpl::setGroup(const fl::string &groupName) {
    mInternal->setGroup(groupName);
}

void JsonAudioImpl::Updater::init(JsonAudioImpl *owner) {
    mOwner = owner;
    fl::EngineEvents::addListener(this);
}

JsonAudioImpl::Updater::~Updater() { fl::EngineEvents::removeListener(this); }

void JsonAudioImpl::Updater::onPlatformPreLoop2() {}

void JsonAudioImpl::toJson(FLArduinoJson::JsonObject &json) const {
    json["name"] = name();
    json["group"] = mInternal->groupName().c_str();
    json["type"] = "audio";
    json["id"] = mInternal->id();
    if (!mSerializeBuffer.empty()) {
        json["audioData"] = mSerializeBuffer.c_str();
    }
}

static bool isdigit(char c) { return c >= '0' && c <= '9'; }
static bool isspace(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r';
}

struct AudioBuffer {
    fl::vector<int16_t> samples;
    uint32_t timestamp = 0;
};

// Fast manual parsing of PCM data from a samples array string
// Input: samples string like "[1,2,3,-4,5]"
// Output: fills the samples vector
static void parsePcmSamplesString(const fl::string &samplesStr, fl::vector<int16_t> *samples) {
    samples->clear();
    
    size_t i = 0, n = samplesStr.size();
    
    // Find opening '['
    while (i < n && samplesStr[i] != '[')
        ++i;
    if (i == n) return; // no array found
    ++i; // skip '['
    
    while (i < n && samplesStr[i] != ']') {
        // Skip whitespace
        while (i < n && isspace(samplesStr[i]))
            ++i;
        if (i >= n || samplesStr[i] == ']') break;
        
        // Parse number
        bool negative = false;
        if (samplesStr[i] == '-') {
            negative = true;
            ++i;
        } else if (samplesStr[i] == '+') {
            ++i;
        }
        
        int value = 0;
        bool hasDigits = false;
        while (i < n && isdigit(static_cast<unsigned char>(samplesStr[i]))) {
            hasDigits = true;
            value = value * 10 + (samplesStr[i] - '0');
            ++i;
        }
        
        if (hasDigits) {
            if (negative) value = -value;
            samples->push_back(static_cast<int16_t>(value));
        }
        
        // Skip whitespace and comma
        while (i < n && (isspace(samplesStr[i]) || samplesStr[i] == ','))
            ++i;
    }
}

static void parseJsonToAudioBuffers(const FLArduinoJson::JsonVariantConst &jsonValue,
                                    fl::vector<AudioBuffer> *audioBuffers) {
    audioBuffers->clear();
    
    // Use JSON parser to extract array of audio buffer objects
    if (!jsonValue.is<FLArduinoJson::JsonArrayConst>()) {
        return;
    }
    
    FLArduinoJson::JsonArrayConst array = jsonValue.as<FLArduinoJson::JsonArrayConst>();
    
    for (FLArduinoJson::JsonVariantConst item : array) {
        if (!item.is<FLArduinoJson::JsonObjectConst>()) {
            continue;
        }
        
        FLArduinoJson::JsonObjectConst obj = item.as<FLArduinoJson::JsonObjectConst>();
        AudioBuffer buffer;
        buffer.timestamp = 0; // Initialize timestamp to prevent uninitialized warning
        
        // Use JSON parser to extract timestamp using proper type checking
        auto timestampVar = obj["timestamp"];
        if (fl::getJsonType(timestampVar) == fl::JSON_INTEGER) {
            buffer.timestamp = timestampVar.as<uint32_t>();
        }
        
        // Use JSON parser to extract samples array as string, then parse manually
        auto samplesVar = obj["samples"];
        if (fl::getJsonType(samplesVar) == fl::JSON_ARRAY) {
            fl::string& samplesStr = scratchBuffer();
            samplesStr.clear();
            serializeJson(samplesVar, samplesStr);
            parsePcmSamplesString(samplesStr, &buffer.samples);
        }
        
        // Add buffer if it has samples
        if (!buffer.samples.empty()) {
            audioBuffers->push_back(fl::move(buffer));
        }
    }
}

void JsonAudioImpl::updateInternal(const fl::Json &json) {
    // For complex parsing, we need to access the underlying FLArduinoJson variant
    // This is acceptable since it's isolated within the implementation
    const ::FLArduinoJson::JsonVariant& value = json.variant();
    
    // FASTLED_WARN("Unimplemented jsAudioImpl::updateInternal");
    mSerializeBuffer.clear();
    serializeJson(value, mSerializeBuffer);
    
    // Parse audio buffers with timestamps using hybrid JSON/manual parsing
    fl::vector<AudioBuffer> audioBuffers;
    parseJsonToAudioBuffers(value, &audioBuffers);
    
    // Convert each audio buffer to a single AudioSample object (no chunking)
    for (const auto& buffer : audioBuffers) {
        const fl::vector<int16_t>& samples = buffer.samples;
        uint32_t timestamp = buffer.timestamp;
        
        // Create one AudioSample per buffer object (preserve separation)
        if (!samples.empty()) {
            AudioSampleImplPtr sample = fl::make_shared<AudioSampleImpl>();
            sample->assign(samples.begin(), samples.end(), timestamp);
            mAudioSampleImpls.push_back(sample);
            
            // Maintain buffer limit to prevent excessive accumulation
            while (mAudioSampleImpls.size() > 10) {
                mAudioSampleImpls.erase(mAudioSampleImpls.begin());
            }
        }
    }
}

AudioSample JsonAudioImpl::next() {
    AudioSampleImplPtr out;
    if (mAudioSampleImpls.empty()) {
        // FASTLED_WARN("No audio samples available");
        return out;
    }
    // auto sample = mAudioSampleImpls.back();
    // mAudioSampleImpls.pop_back();
    out = mAudioSampleImpls.front();
    mAudioSampleImpls.erase(mAudioSampleImpls.begin());
    // FASTLED_WARN("Returning audio sample of size " << out->pcm().size());
    return AudioSample(out);
}

bool JsonAudioImpl::hasNext() { return !mAudioSampleImpls.empty(); }

} // namespace fl

#endif // FASTLED_ENABLE_JSON
