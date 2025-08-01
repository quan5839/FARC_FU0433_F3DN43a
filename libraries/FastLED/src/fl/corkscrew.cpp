#include "fl/corkscrew.h"
#include "fl/algorithm.h"
#include "fl/assert.h"
#include "fl/math.h"
#include "fl/splat.h"
#include "fl/warn.h"
#include "fl/tile2x2.h"
#include "fl/math_macros.h"
#include "fl/unused.h"
#include "fl/map_range.h"
#include "fl/leds.h"
#include "fl/grid.h"
#include "fl/screenmap.h"
#include "fl/int.h"




namespace fl {

namespace {

// New helper function to calculate individual LED position
vec2f calculateLedPositionExtended(fl::u16 ledIndex, fl::u16 numLeds, float totalTurns, const Gap& gapParams, fl::u16 width, fl::u16 height) {
    FL_UNUSED(height);
    FL_UNUSED(totalTurns);

    // Check if gap feature is active AND will actually be triggered
    bool gapActive = (gapParams.num_leds > 0 && gapParams.gap > 0.0f && numLeds > static_cast<fl::u16>(gapParams.num_leds));
    
    if (!gapActive) {
        // Original behavior when no gap or gap never triggers
        const float ledProgress = static_cast<float>(ledIndex) / static_cast<float>(numLeds - 1);
        const fl::u16 row = ledIndex / width;
        const fl::u16 remainder = ledIndex % width;
        const float alpha = static_cast<float>(remainder) / static_cast<float>(width);
        const float width_pos = ledProgress * numLeds;
        const float height_pos = static_cast<float>(row) + alpha;
        return vec2f(width_pos, height_pos);
    }
    
    // Simplified gap calculation based on user expectation
    // User wants: LED0=0, LED1=3, LED2=6(wraps to 0) with width=3
    // This suggests they want regular spacing of width units per LED
    
    // Simple spacing: each LED is separated by exactly width units  
    float width_pos = static_cast<float>(ledIndex) * static_cast<float>(width);
    
    // For height, divide by width to get turn progress
    float height_pos = width_pos / static_cast<float>(width);
    
    return vec2f(width_pos, height_pos);
}

void generateState(const Corkscrew::Input &input, CorkscrewState *output) {
    output->width = input.calculateWidth();
    output->height = input.calculateHeight();
    // No longer pre-calculating mapping - positions computed on-the-fly
}
}  // namespace


Corkscrew::Corkscrew(const Corkscrew::Input &input) : mInput(input) {
    fl::generateState(mInput, &mState);
}

vec2f Corkscrew::at_no_wrap(fl::u16 i) const {
    if (i >= mInput.numLeds) {
        // Handle out-of-bounds access, possibly by returning a default value
        return vec2f(0, 0);
    }
    
    // Compute position on-the-fly
    vec2f position = calculateLedPositionExtended(i, mInput.numLeds, mInput.totalTurns, 
                                         mInput.gapParams, mState.width, mState.height);
    
    // // Apply inversion if requested
    // if (mInput.invert) {
    //     fl::u16 invertedIndex = mInput.numLeds - 1 - i;
    //     position = calculateLedPositionExtended(invertedIndex, mInput.numLeds, mInput.totalTurns, 
    //                                    mInput.gapParams, mState.width, mState.height);
    // }

    // now wrap the x-position
    //position.x = fmodf(position.x, static_cast<float>(mState.width));
    
    return position;
}

vec2f Corkscrew::at_exact(fl::u16 i) const {
    // Get the unwrapped position
    vec2f position = at_no_wrap(i);
    
    // Apply cylindrical wrapping to the x-position (like at_wrap does)
    position.x = fmodf(position.x, static_cast<float>(mState.width));
    
    return position;
}


Tile2x2_u8 Corkscrew::at_splat_extrapolate(float i) const {
    if (i >= mInput.numLeds) {
        // Handle out-of-bounds access, possibly by returning a default
        // Tile2x2_u8
        FASTLED_ASSERT(false, "Out of bounds access in Corkscrew at_splat: "
                                  << i << " size: " << mInput.numLeds);
        return Tile2x2_u8();
    }
    
    // Use the splat function to convert the vec2f to a Tile2x2_u8
    float i_floor = floorf(i);
    float i_ceil = ceilf(i);
    if (ALMOST_EQUAL_FLOAT(i_floor, i_ceil)) {
        // If the index is the same, just return the splat of that index
        vec2f position = at_no_wrap(static_cast<fl::u16>(i_floor));
        return splat(position);
    } else {
        // Interpolate between the two points and return the splat of the result
        vec2f pos1 = at_no_wrap(static_cast<fl::u16>(i_floor));
        vec2f pos2 = at_no_wrap(static_cast<fl::u16>(i_ceil));
        float t = i - i_floor;
        vec2f interpolated_pos = map_range(t, 0.0f, 1.0f, pos1, pos2);
        return splat(interpolated_pos);
    }
}

fl::size Corkscrew::size() const { return mInput.numLeds; }

Corkscrew::State Corkscrew::generateState(const Corkscrew::Input &input) {
    CorkscrewState output;
    fl::generateState(input, &output);
    return output;
}

Tile2x2_u8_wrap Corkscrew::at_wrap(float i) const {
    if (mCachingEnabled) {
        // Use cache if enabled
        initializeCache();
        
        // Convert float index to integer for cache lookup
        fl::size cache_index = static_cast<fl::size>(i);
        if (cache_index < mTileCache.size()) {
            return mTileCache[cache_index];
        }
    }
    
    // Fall back to dynamic calculation if cache disabled or index out of bounds
    return calculateTileAtWrap(i);
}

Tile2x2_u8_wrap Corkscrew::calculateTileAtWrap(float i) const {
    // This is a splatted pixel, but wrapped around the cylinder.
    // This is useful for rendering the corkscrew in a cylindrical way.
    Tile2x2_u8 tile = at_splat_extrapolate(i);
    Tile2x2_u8_wrap::Entry data[2][2];
    vec2i16 origin = tile.origin();
    for (fl::u8 x = 0; x < 2; x++) {
        for (fl::u8 y = 0; y < 2; y++) {
            // For each pixel in the tile, map it to the cylinder so that each subcomponent
            // is mapped to the correct position on the cylinder.
            vec2i16 pos = origin + vec2i16(x, y);
            // now wrap the x-position
            pos.x = fmodf(pos.x, static_cast<float>(mState.width));
            data[x][y] = {pos, tile.at(x, y)};
        }
    }
    return Tile2x2_u8_wrap(data);
}

void Corkscrew::setCachingEnabled(bool enabled) {
    if (!enabled && mCachingEnabled) {
        // Caching was enabled, now disabling - clear the cache
        mTileCache.clear();
        mCacheInitialized = false;
    }
    mCachingEnabled = enabled;
}

void Corkscrew::initializeCache() const {
    if (!mCacheInitialized && mCachingEnabled) {
        // Initialize cache with tiles for each LED position
        mTileCache.resize(mInput.numLeds);
        
        // Populate cache lazily
        for (fl::size i = 0; i < mInput.numLeds; ++i) {
            mTileCache[i] = calculateTileAtWrap(static_cast<float>(i));
        }
        
        mCacheInitialized = true;
    }
}

// New rectangular buffer functionality implementation

void Corkscrew::initializeBuffer() const {
    if (!mBufferInitialized) {
        fl::size buffer_size = static_cast<fl::size>(mState.width) * static_cast<fl::size>(mState.height);
        mCorkscrewLeds.resize(buffer_size, CRGB::Black);
        mBufferInitialized = true;
    }
}

fl::vector<CRGB>& Corkscrew::getBuffer() {
    initializeBuffer();
    return mCorkscrewLeds;
}

const fl::vector<CRGB>& Corkscrew::getBuffer() const {
    initializeBuffer();
    return mCorkscrewLeds;
}

CRGB* Corkscrew::data() {
    initializeBuffer();
    return mCorkscrewLeds.data();
}

const CRGB* Corkscrew::data() const {
    initializeBuffer();
    return mCorkscrewLeds.data();
}

void Corkscrew::readFrom(const fl::Grid<CRGB>& source_grid, bool use_multi_sampling) {

    if (use_multi_sampling) {
        readFromMulti(source_grid);
        return;
    }


    // Initialize the buffer if not already done
    initializeBuffer();
    
    // Clear buffer first
    clearBuffer();
    
    // Iterate through each LED in the corkscrew
    for (fl::size led_idx = 0; led_idx < mInput.numLeds; ++led_idx) {
        // Get the rectangular coordinates for this corkscrew LED
        vec2f rect_pos = at_no_wrap(static_cast<fl::u16>(led_idx));
        
        // Convert to integer coordinates for indexing
        vec2i16 coord(static_cast<fl::i16>(rect_pos.x + 0.5f), 
                      static_cast<fl::i16>(rect_pos.y + 0.5f));
        
        // Clamp coordinates to grid bounds
        coord.x = MAX(0, MIN(coord.x, static_cast<fl::i16>(source_grid.width()) - 1));
        coord.y = MAX(0, MIN(coord.y, static_cast<fl::i16>(source_grid.height()) - 1));
        
        // Sample from the source fl::Grid using its at() method
        CRGB sampled_color = source_grid.at(coord.x, coord.y);
        
        // Store the sampled color directly at the LED index position
        mCorkscrewLeds[led_idx] = sampled_color;
    }
}

void Corkscrew::clearBuffer() {
    initializeBuffer();
    for (fl::size i = 0; i < mCorkscrewLeds.size(); ++i) {
        mCorkscrewLeds[i] = CRGB::Black;
    }
}

void Corkscrew::fillBuffer(const CRGB& color) {
    initializeBuffer();
    for (fl::size i = 0; i < mCorkscrewLeds.size(); ++i) {
        mCorkscrewLeds[i] = color;
    }
}

void Corkscrew::readFromMulti(const fl::Grid<CRGB>& source_grid) const {
    // Ensure buffer is initialized
    initializeBuffer();
    
    // Clear buffer first
    const_cast<Corkscrew*>(this)->clearBuffer();
    
    // Iterate through each LED in the corkscrew
    for (fl::size led_idx = 0; led_idx < mInput.numLeds; ++led_idx) {
        // Get the wrapped tile for this LED position
        Tile2x2_u8_wrap tile = at_wrap(static_cast<float>(led_idx));
        
        // Accumulate color from the 4 sample points with their weights
        fl::u32 r_accum = 0, g_accum = 0, b_accum = 0;
        fl::u32 total_weight = 0;
        
        // Sample from each of the 4 corners of the tile
        for (fl::u8 x = 0; x < 2; x++) {
            for (fl::u8 y = 0; y < 2; y++) {
                const auto& entry = tile.at(x, y);
                vec2i16 pos = entry.first;   // position is the first element of the pair
                fl::u8 weight = entry.second; // weight is the second element of the pair
                
                // Bounds check for the source grid
                if (pos.x >= 0 && pos.x < static_cast<fl::i16>(source_grid.width()) && 
                    pos.y >= 0 && pos.y < static_cast<fl::i16>(source_grid.height())) {
                    
                    // Sample from the source grid
                    CRGB sample_color = source_grid.at(pos.x, pos.y);
                    
                    // Accumulate weighted color components
                    r_accum += static_cast<fl::u32>(sample_color.r) * weight;
                    g_accum += static_cast<fl::u32>(sample_color.g) * weight;
                    b_accum += static_cast<fl::u32>(sample_color.b) * weight;
                    total_weight += weight;
                }
            }
        }
        
        // Calculate final color by dividing by total weight
        CRGB final_color = CRGB::Black;
        if (total_weight > 0) {
            final_color.r = static_cast<fl::u8>(r_accum / total_weight);
            final_color.g = static_cast<fl::u8>(g_accum / total_weight);
            final_color.b = static_cast<fl::u8>(b_accum / total_weight);
        }
        
        // Store the result in the LED buffer at the LED index position
        if (led_idx < mCorkscrewLeds.size()) {
            mCorkscrewLeds[led_idx] = final_color;
        }
    }
}

// Iterator implementation
vec2f CorkscrewState::iterator::operator*() const {
    return corkscrew_->at_no_wrap(static_cast<fl::u16>(position_));
}

fl::ScreenMap Corkscrew::toScreenMap(float diameter) const {
    // Create a ScreenMap with the correct number of LEDs
    fl::ScreenMap screenMap(mInput.numLeds, diameter);
    
    // For each LED index, calculate its position and set it in the ScreenMap
    for (fl::u16 i = 0; i < mInput.numLeds; ++i) {
        // Get the wrapped 2D position for this LED index in the cylindrical mapping
        vec2f position = at_exact(i);
        
        // Set the wrapped position in the ScreenMap
        screenMap.set(i, position);
    }
    
    return screenMap;
}

} // namespace fl
