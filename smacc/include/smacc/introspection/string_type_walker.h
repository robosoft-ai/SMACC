
#pragma once
#include <string>
#include <memory>
#include <vector>
#include <typeinfo>
#include <map>

namespace smacc
{
namespace introspection
{
class TypeInfo
{
public:
    typedef std::shared_ptr<TypeInfo> Ptr;

    std::vector<Ptr> templateParameters;

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
        return this->finaltype.substr(0, index);
    }

    static TypeInfo::Ptr getTypeInfoFromString(std::string inputtext);
    static TypeInfo::Ptr getTypeInfoFromTypeid(const std::type_info &tid);

    template <typename T>
    static TypeInfo::Ptr getTypeInfoFromType()
    {
        return TypeInfo::getTypeInfoFromTypeid(typeid(T));
    }

    const std::string &getFullName()
    {
        return this->finaltype;
    }

    static std::map<std::string, Ptr> typeInfoDatabase;

private:
    std::string tkey;
    std::string codedtype;
    std::string finaltype;
};
} // namespace introspection
} // namespace smacc