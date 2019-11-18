#include <smacc/string_type_walker.h>

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
bool replace(std::string &str, const std::string &from, const std::string &to)
{
    size_t start_pos = str.find(from);
    if (start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

std::string replace_back(std::string roottype, std::map<std::string, std::string> &typesdict)
{
    while (roottype.find("$") != std::string::npos)
    {
        for (auto &t : typesdict)
        {
            auto &tkey = t.first;
            auto &tval = t.second;

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

smacc::TypeInfo::Ptr TypeInfo::getTypeInfoFromTypeid(const std::type_info &tid)
{
    return TypeInfo::getTypeInfoFromString(demangleSymbol(tid.name()));
}

smacc::TypeInfo::Ptr TypeInfo::getTypeInfoFromString(std::string inputtext)
{
    auto it = typeInfoDatabase.find(inputtext);
    
    if(it != typeInfoDatabase.end())
        return typeInfoDatabase[inputtext];

    bool ok = false;
    int typecount = 0;
    std::string originalinputtext = inputtext;
    std::map<std::string, std::string> typesdict;

    while (!ok)
    {
        //simpletypeRE = r"[^<>,\s]+<[^<>]+>"
        //print ("input: " + inputtext)

        const char *simpletypeRE = "[^\\<\\>,\\s]+\\<[^\\<\\>]+\\>";
        //std::cout << inputtext << std::endl;

        std::smatch matches;
        std::regex regex(simpletypeRE); // matches words beginning by "sub"

        //matches = [m for m in enumerate(re.finditer(simpletypeRE, inputtext))]
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

        int i = 0;
        for (auto &m : matches)
        {
            std::string tstr = m;
            auto tkey = "$T" + std::to_string(typecount);
            replace(inputtext, tstr, tkey);

            //std::cout << "updating input text:" << inputtext << std::endl;

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
    //         if not "$" in b:
    //             allbasetypes.add(b)

    std::set<std::string> allbasetypes;
    for (auto &tkey : typesdict)
    {
        auto flat = tkey.second;
        //std::cout << flat << std::endl;

        size_t startindex = flat.find("<");
        size_t endindex = flat.find(">");
        if (startindex != std::string::npos)
        {
            flat = flat.substr(startindex + 1, endindex - startindex - 1);
            std::vector<std::string> basetypes;

            std::istringstream iss(flat);
            std::string token;
            while (std::getline(iss, token, ','))
            {
                boost::trim(token);
                basetypes.push_back(token);
            }

            for (auto &b : basetypes)
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

        for (auto &b : allbasetypes)
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

    //std::cout << "---------- TYPES -------" << std::endl;
    std::vector<smacc::TypeInfo::Ptr> types;
    for (auto &t : typesdict)
    {
        auto &tkey = t.first;
        auto &tval = t.second;
        auto finaltype = replace_back(tval, typesdict);
        auto tinfo = std::make_shared<TypeInfo>(tkey, tval, finaltype);
        types.push_back(tinfo);

        //std::cout << tinfo->toString() << std::endl;
    }

    smacc::TypeInfo::Ptr roottype = nullptr;
    for (auto &t : types)
    {
        if (t->finaltype == originalinputtext)
        {
            roottype = t;
            break;
        }
    }

    for (auto &t : types)
    {
        std::vector<std::pair<int, smacc::TypeInfo::Ptr>> unorderedTemplateParameters;
        for (auto &t2 : types)
        {
            if (t == t2)
                continue;

            auto index = t->codedtype.find(t2->tkey);
            if (index != std::string::npos)
            {
                auto pair = std::make_pair(index, t2);
                unorderedTemplateParameters.push_back(pair);
            }
        }

        std::sort(unorderedTemplateParameters.begin(), unorderedTemplateParameters.end(),
                  [](auto &a, auto &b) -> bool {
                      return a.first <= b.first;
                  });

        ROS_ERROR_STREAM("------------------");
        ROS_ERROR_STREAM("CREATING TYPE:" << t->getFullName());

        for (auto &item : unorderedTemplateParameters)
        {
            ROS_ERROR_STREAM(" - template paramter: " << item.second->getFullName());
            t->templateParameters.push_back(item.second);
        }
        ROS_ERROR_STREAM("------------------");
    }

    ROS_ERROR_STREAM("ADDING TYPE TO DATABASE: "<< inputtext);
    ROS_ERROR_STREAM("Current Database");
    for(auto& en : typeInfoDatabase)
    {
        ROS_ERROR_STREAM("- " << en.first);
    }

    typeInfoDatabase[originalinputtext] = roottype;
    return roottype;
}

/*
  
    print (typesdict)
    roottype = [t for t in types if t.finaltype == originalinputtext][0]

    print "---------------------------------"

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

} // namespace smacc
