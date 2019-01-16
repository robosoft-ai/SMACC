#pragma once
#include <boost/any.hpp>
#include <map>
#include <map>
#include <string>

namespace smacc {
using namespace std;

///
/// \brief UserData
///
typedef map<string, boost::any> UserData;

///
/// \brief The AddStateParameters struct
///
struct AddStateParameters {
    map<string, string> transitions;
    map<string, string> remapping;
};
}
