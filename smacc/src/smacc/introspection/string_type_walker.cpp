
/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

#include <smacc/introspection/string_type_walker.h>

#include <regex>
#include <memory>
#include <map>
#include <set>
#include <iostream>
#include <boost/algorithm/string.hpp>

#include <boost/algorithm/string/find_iterator.hpp>
#include <algorithm>
#include <boost/algorithm/string/trim.hpp>

#include <smacc/common.h>

namespace smacc
{
namespace introspection
{
bool replace(std::string& str, const std::string& from, const std::string& to)
{
  size_t start_pos = str.find(from);
  if (start_pos == std::string::npos)
    return false;
  str.replace(start_pos, from.length(), to);
  return true;
}

std::string replace_back(std::string roottype, std::vector<std::pair<std::string, std::string>>& orderedtypesdict)
{
  while (roottype.find("$") != std::string::npos)
  {
    for (auto& t : orderedtypesdict)
    {
      auto& tkey = t.first;
      auto& tval = t.second;

      replace(roottype, tkey, tval);
    }
  }

  return roottype;

  //   def replace_back(roottype, typesdict):
  //     # replace back
  //     while "$" in roottype:
  //         #print roottype
  //         for tkey in typesdict:
  //             tval = typesdict[tkey]
  //             roottype = roottype.replace(tkey, tval)

  //     return roottype
}

std::map<std::string, TypeInfo::Ptr> TypeInfo::typeInfoDatabase;

TypeInfo::Ptr TypeInfo::getFromStdTypeInfo(const std::type_info& tid)
{
  return TypeInfo::getTypeInfoFromString(demangleSymbol(tid.name()));
}

TypeInfo::Ptr TypeInfo::getTypeInfoFromString(std::string inputtext)
{
  auto it = typeInfoDatabase.find(inputtext);

  if (it != typeInfoDatabase.end())
    return typeInfoDatabase[inputtext];

  bool ok = false;
  int typecount = 0;
  std::string originalinputtext = inputtext;
  std::map<std::string, std::string> typesdict;
  std::map<std::string, std::string> typesdict_content;

  while (!ok)
  {
    // simpletypeRE = r"[^<>,\s]+<[^<>]+>"
    // print ("input: " + inputtext)

    const char* simpletypeRE = "[^\\<\\>,\\s]+\\<[^\\<\\>]+\\>";
    // std::cout << inputtext << std::endl;

    // locate moste outer template
    std::smatch matches;
    std::regex regex(simpletypeRE);  // matches words beginning by "sub"

    // matches = [m for m in enumerate(re.finditer(simpletypeRE, inputtext))]
    std::regex_search(inputtext, matches, regex);

    //   if len(matches) == 0:
    //     if len(typesdict) == 0:
    //         tkey = "$T" + str(typecount)
    //         typesdict[tkey] = inputtext
    //     break

    if (matches.size() == 0)
    {
      if (typesdict.size() == 0)
      {
        auto tkey = "$T" + std::to_string(typecount);
        typesdict[tkey] = inputtext;
      }
      break;
    }

    // for i, m in matches:
    //     tstr = m.group(0)
    //     tkey = "$T" + str(typecount)
    //     inputtext = inputtext.replace(tstr, tkey)
    //     print("updating input text: " + inputtext)
    //     typecount += 1
    //     typesdict[tkey] = tstr

    // find and replace internal templates with tokens
    int i = 0;
    for (auto& m : matches)
    {
      std::string tstr = m;
      auto tkey = "$T" + std::to_string(typecount);
      replace(inputtext, tstr, tkey);

      // std::cout << "updating input text:" << inputtext << std::endl;

      typecount++;
      typesdict[tkey] = tstr;

      i++;
    }
  }

  // allbasetypes = set()
  // for tkey in typesdict.keys():
  //     flat = typesdict[tkey]
  //     print flat
  //     startindex = flat.index("<")
  //     flat = flat[startindex + 1:-1]
  //     basetypes = [t.strip() for t in flat.split(",")]
  //     for b in basetypes:
  //         if "$" not in b:
  //             allbasetypes.add(b)

  // SORT by token key to replace back in a single pass
  std::vector<std::pair<std::string, std::string>> orderedTypedict;
  for (const auto& item : typesdict)
  {
    orderedTypedict.emplace_back(item);
  }

  std::sort(orderedTypedict.begin(), orderedTypedict.end(),
            [](auto& a, auto& b) { return std::stoi(a.first.substr(2)) < std::stoi(b.first.substr(2)); });

  std::set<std::string> allbasetypes;
  for (auto& typeentry : orderedTypedict /*typesdict*/)
  {
    auto flat = typeentry.second;
    // std::cout << flat << std::endl;

    size_t startindex = flat.find("<");
    size_t endindex = flat.find(">");
    if (startindex != std::string::npos)
    {
      flat = flat.substr(startindex + 1,
                         endindex - startindex - 1);  // gets the list of template parameters in csv format

      // std::cout << typeentry.first <<":" << flat << std::endl;
      typesdict_content[typeentry.first] = flat;
      std::vector<std::string> localbasetypes;

      std::istringstream iss(flat);
      std::string token;
      while (std::getline(iss, token, ','))
      {
        boost::trim(token);
        localbasetypes.push_back(token);
        // std::cout << "base type: " << token << std::endl;
      }

      for (auto& b : localbasetypes)
      {
        size_t found = b.find("$");

        if (found == std::string::npos)
        {
          allbasetypes.insert(b);
        }
      }
    }

    // for b in allbasetypes:
    // typesdict[b] = b

    // refresh
    for (auto& b : allbasetypes)
    {
      typesdict[b] = b;
    }
  }
  // types = []
  // for tkey in typesdict:
  //     finaltype = replace_back(typesdict[tkey], typesdict)
  //     t = TypeInfo(tkey, typesdict[tkey], finaltype)
  //     types.append(t)

  //     print t

  // append leaf types
  for (auto& b : allbasetypes)
  {
    orderedTypedict.push_back({ b, b });
  }

  // std::cout << "---------- TYPES -------" << std::endl;
  std::vector<TypeInfo::Ptr> types;
  std::vector<std::string> tokens;

  for (auto t : orderedTypedict)  // must be ordered. to avoid issue, ie: $11 to be replaced in $T14
  {
    // auto t = *it;
    auto& tkey = t.first;
    auto& tval = t.second;
    auto finaltype = replace_back(tval, orderedTypedict);
    auto tinfo = std::make_shared<TypeInfo>(tkey, tval, finaltype);
    types.push_back(tinfo);
    tokens.push_back(tkey);

    // std::cout << "replacing back: " << finaltype << std::endl;
  }

  TypeInfo::Ptr roottype = nullptr;
  for (auto& t : types)
  {
    // std::cout << t->finaltype << " vs " <<originalinputtext;
    if (t->finaltype == originalinputtext)
    {
      roottype = t;
      break;
    }
  }

  // std::sort(types.begin(), types.end(),[](auto& a, auto& b)
  // {
  //     return a->getFullName().size() > b->getFullName().size();
  // });

  /*
    std::cout<<"types order:" << std::endl;
    for(auto t: types)
    {
        std::cout<< t->codedtype << std::endl;
    }
    std::cout<<"---------" << std::endl;

    std::cout<<"types order:" << std::endl;
    for(auto t: types)
    {
        std::cout<< t->finaltype << std::endl;
    }
    std::cout<<"---------" << std::endl;
    */

  for (size_t i = 0; i < types.size(); i++)
  {
    auto t = types[i];
    auto ttoken = tokens[i];

    // auto t = types[it1];
    std::vector<std::pair<int, TypeInfo::Ptr>> unorderedTemplateParameters;

    // std::cout << "original typestr: " << codedtypecopy << std::endl;

    // auto codedtypecopy = t->codedtype;
    auto codedtypecopy = typesdict_content[ttoken];

    // std::cout << "original typestr: " << codedtypecopy << std::endl;
    // std::cout << "original token: " << ttoken << std::endl;
    // std::cout << "original typestr: " << typesdict_content[ttoken] << std::endl;

    // size_t startindex = codedtypecopy.find("<");
    // size_t endindex = codedtypecopy.find(">");

    // if (startindex != std::string::npos)
    // {
    //     codedtypecopy = codedtypecopy.substr(startindex + 1, endindex - startindex - 1);
    // }
    // std::cout << "original typestr: " << codedtypecopy << std::endl;

    for (auto& t2 : types)
    // for (auto it2= types.size() -1; it2 >=0; it2--)
    {
      // std::cout << it2 << std::endl;
      // auto t2 = types[it2];
      // std::cout << t2->getFullName() << std::endl;
      if (t == t2)
        continue;

      auto index = codedtypecopy.find(t2->tkey);
      if (index != std::string::npos)
      {
        auto pair = std::make_pair(index, t2);  // this line is important for the order of templates
        // auto pair = std::make_pair(0, t2);
        // std::cout << "matches: " << t2->tkey <<std::endl;
        unorderedTemplateParameters.push_back(pair);
        replace(codedtypecopy, t2->tkey, "");  // consume token
        // std::cout << "codedtypecopy: " << codedtypecopy << std::endl;
      }
    }

    std::sort(unorderedTemplateParameters.begin(), unorderedTemplateParameters.end(),
              [](auto& a, auto& b) -> bool { return a.first <= b.first; });

    ROS_DEBUG_STREAM("------------------");
    ROS_DEBUG_STREAM("CREATING TYPE:" << t->getFullName());

    for (auto& item : unorderedTemplateParameters)
    {
      ROS_DEBUG_STREAM(" - template parameter: " << item.second->getFullName());
      t->templateParameters.push_back(item.second);
    }
    ROS_DEBUG_STREAM("------------------");
  }

  ROS_DEBUG_STREAM("ADDING TYPE TO DATABASE: " << inputtext);
  ROS_DEBUG_STREAM("Current Database");
  for (auto& en : typeInfoDatabase)
  {
    ROS_DEBUG_STREAM("- " << en.first);
  }

  typeInfoDatabase[originalinputtext] = roottype;
  return roottype;
}

/*

    print (typesdict)
    roottype = [t for t in types if t.finaltype == originalinputtext][0]

    print("---------------------------------")

    # fill template parameters
    for t in types:
        for t2 in types:
            if t2.tkey in t.codedtype:
                index = t.codedtype.index(t2.tkey)
                t.template_parameters.append((index, t2))

        t.template_parameters = [x[1] for x in sorted(
            t.template_parameters, key=lambda e: e[0])]

    return roottype

}
*/

}  // namespace introspection
}  // namespace smacc
