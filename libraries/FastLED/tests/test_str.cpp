
// g++ --std=c++11 test.cpp

#include "test.h"

#include "test.h"
#include "fl/str.h"
#include "fl/vector.h"
#include "crgb.h"
#include <sstream>

#include "fl/namespace.h"

using namespace fl;

TEST_CASE("Str basic operations") {
    SUBCASE("Construction and assignment") {
        Str s1;
        CHECK(s1.size() == 0);
        CHECK(s1.c_str()[0] == '\0');

        Str s2("hello");
        CHECK(s2.size() == 5);
        CHECK(strcmp(s2.c_str(), "hello") == 0);

        Str s3 = s2;
        CHECK(s3.size() == 5);
        CHECK(strcmp(s3.c_str(), "hello") == 0);

        s1 = "world";
        CHECK(s1.size() == 5);
        CHECK(strcmp(s1.c_str(), "world") == 0);
    }

    SUBCASE("Comparison operators") {
        Str s1("hello");
        Str s2("hello");
        Str s3("world");

        CHECK(s1 == s2);
        CHECK(s1 != s3);
    }

    SUBCASE("Indexing") {
        Str s("hello");
        CHECK(s[0] == 'h');
        CHECK(s[4] == 'o');
        CHECK(s[5] == '\0');  // Null terminator
    }

    SUBCASE("Append") {
        Str s("hello");
        s.append(" world");
        CHECK(s.size() == 11);
        CHECK(strcmp(s.c_str(), "hello world") == 0);
    }

    SUBCASE("CRGB to Str") {
        CRGB c(255, 0, 0);
        Str s = c.toString();
        CHECK_EQ(s, "CRGB(255,0,0)");
    }

    SUBCASE("Copy-on-write behavior") {
        Str s1("hello");
        Str s2 = s1;
        s2.append(" world");
        CHECK(strcmp(s1.c_str(), "hello") == 0);
        CHECK(strcmp(s2.c_str(), "hello world") == 0);
    }
}


TEST_CASE("Str::reserve") {
    Str s;
    s.reserve(10);
    CHECK(s.size() == 0);
    CHECK(s.capacity() >= 10);

    s.reserve(5);
    CHECK(s.size() == 0);
    CHECK(s.capacity() >= 10);

    s.reserve(500);
    CHECK(s.size() == 0);
    CHECK(s.capacity() >= 500);
    // s << "hello";
    s.append("hello");
    CHECK(s.size() == 5);
    CHECK_EQ(s, "hello");
}

TEST_CASE("Str with fl::FixedVector") {
    fl::FixedVector<Str, 10> vec;
    vec.push_back(Str("hello"));
    vec.push_back(Str("world"));

    CHECK(vec.size() == 2);
    CHECK(strcmp(vec[0].c_str(), "hello") == 0);
    CHECK(strcmp(vec[1].c_str(), "world") == 0);
}

TEST_CASE("Str with long strings") {
    const char* long_string = "This is a very long string that exceeds the inline buffer size and should be allocated on the heap";
    Str s(long_string);
    CHECK(s.size() == strlen(long_string));
    CHECK(strcmp(s.c_str(), long_string) == 0);

    Str s2 = s;
    CHECK(s2.size() == strlen(long_string));
    CHECK(strcmp(s2.c_str(), long_string) == 0);

    s2.append(" with some additional text");
    CHECK(strcmp(s.c_str(), long_string) == 0);  // Original string should remain unchanged
}

TEST_CASE("Str overflowing inline data") {
    SUBCASE("Construction with long string") {
        std::string long_string(FASTLED_STR_INLINED_SIZE + 10, 'a');  // Create a string longer than the inline buffer
        Str s(long_string.c_str());
        CHECK(s.size() == long_string.length());
        CHECK(strcmp(s.c_str(), long_string.c_str()) == 0);
    }

    SUBCASE("Appending to overflow") {
        Str s("Short string");
        std::string append_string(FASTLED_STR_INLINED_SIZE, 'b');  // String to append that will cause overflow
        s.append(append_string.c_str());
        CHECK(s.size() == strlen("Short string") + append_string.length());
        CHECK(s[0] == 'S');
        CHECK(s[s.size() - 1] == 'b');
    }

    SUBCASE("Copy on write with long string") {
        std::string long_string(FASTLED_STR_INLINED_SIZE + 20, 'c');
        Str s1(long_string.c_str());
        Str s2 = s1;
        CHECK(s1.size() == s2.size());
        CHECK(strcmp(s1.c_str(), s2.c_str()) == 0);

        s2.append("extra");
        CHECK(s1.size() == long_string.length());
        CHECK(s2.size() == long_string.length() + 5);
        CHECK(strcmp(s1.c_str(), long_string.c_str()) == 0);
        CHECK(s2[s2.size() - 1] == 'a');
    }
}

TEST_CASE("String concatenation operators") {
    SUBCASE("String literal + fl::to_string") {
        // Test the specific case mentioned in the user query
        fl::string val = "string" + fl::to_string(5);
        CHECK(strcmp(val.c_str(), "string5") == 0);
    }

    SUBCASE("fl::to_string + string literal") {
        fl::string val = fl::to_string(10) + " is a number";
        CHECK(strcmp(val.c_str(), "10 is a number") == 0);
    }

    SUBCASE("String literal + fl::string") {
        fl::string str = "world";
        fl::string result = "Hello " + str;
        CHECK(strcmp(result.c_str(), "Hello world") == 0);
    }

    SUBCASE("fl::string + string literal") {
        fl::string str = "Hello";
        fl::string result = str + " world";
        CHECK(strcmp(result.c_str(), "Hello world") == 0);
    }

    SUBCASE("fl::string + fl::string") {
        fl::string str1 = "Hello";
        fl::string str2 = "World";
        fl::string result = str1 + " " + str2;
        CHECK(strcmp(result.c_str(), "Hello World") == 0);
    }

    SUBCASE("Complex concatenation") {
        fl::string result = "Value: " + fl::to_string(42) + " and " + fl::to_string(3.14f);
        // Check that it contains the expected parts rather than exact match
        CHECK(result.find("Value: ") != fl::string::npos);
        CHECK(result.find("42") != fl::string::npos);
        CHECK(result.find("and") != fl::string::npos);
        CHECK(result.find("3.14") != fl::string::npos);
    }

    SUBCASE("Number + string literal") {
        fl::string result = fl::to_string(100) + " percent";
        CHECK(strcmp(result.c_str(), "100 percent") == 0);
    }

    SUBCASE("String literal + number") {
        fl::string result = "Count: " + fl::to_string(7);
        CHECK(strcmp(result.c_str(), "Count: 7") == 0);
    }
}
