
#pragma once
#include <string>
#include <memory>
#include <vector>
#include <typeinfo>

namespace smacc
{
class TypeInfo
{
public:
    std::string tkey;
    std::string codedtype;
    std::string finaltype;
    std::vector<std::shared_ptr<TypeInfo>> templateParameters;

    TypeInfo(std::string tkey, std::string codedtype, std::string finaltype)
    {
        this->tkey = tkey;
        this->codedtype = codedtype;
        this->finaltype = finaltype;
    }

    std::string toString()
    {
        return this->tkey + ":" + this->finaltype;
    }

    std::string getNonTemplatetypename()
    {
        auto index = this->finaltype.find("<");
        return this->finaltype.substr(0,index);
    }

    static std::shared_ptr<TypeInfo> getTypeInfoFromString(std::string inputtext);
    static std::shared_ptr<TypeInfo> getTypeInfoFromTypeid(const std::type_info& tid);  
};
}