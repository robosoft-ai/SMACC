#include "nasa_robodyn_mechanisms_core/JointCommandFactory.h"

using namespace std;
using namespace log4cpp;
namespace RSU = StringUtilities;

/***************************************************************************//**
 *
 * @brief Generate Joint command based on the type of joint
 *
 * @param mechanism
 * @param io
 * @throw runtime_error if the jointType is not valid
 * @return Pointer to the interface
 *
 ******************************************************************************/
JointCommandInterfacePtr JointCommandFactory::generate(const std::string& mechanism, const JointCommandInterface::IoFunctions& io)
{
    JointCommandInterfacePtr interfacePtr;

    string jointType;

    jointType = Private::getJointTypeFromParameterFile(io.getCommandFile(mechanism));

    if (jointType == "SeriesElastic")
    {
        interfacePtr = boost::make_shared<JointCommandSeriesElastic>(mechanism, io);
    }

    else if (jointType == "Rigid")
    {
        interfacePtr = boost::make_shared<JointCommandRigid>(mechanism, io);
    }

    else if (jointType == "Gripper")
    {
        interfacePtr = boost::make_shared<JointCommandGripper>(mechanism, io);
    }

    //! @todo implement primary finger interface
    else if (jointType == "PrimaryFinger")
    {
        interfacePtr = boost::make_shared<JointCommandFinger<3> >(mechanism, io);
    }

    //! @todo implement secondar fingers interface
    else if (jointType == "SecondaryFingers")
    {
        interfacePtr = boost::make_shared<JointCommandFinger<2> >(mechanism, io);
    }

    //! @todo implement thumb interface
    else if (jointType == "Thumb")
    {
        interfacePtr = boost::make_shared<JointCommandFinger<4> >(mechanism, io);
    }

    else if (jointType == "Wrist")
    {
        interfacePtr = boost::make_shared<JointCommandWrist>(mechanism, io);
    }

    else
    {
        stringstream err;
        err << "Unsupported joint type [" << jointType << "] found in CommandFile";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }

    return interfacePtr;
}

/***************************************************************************//**
 *
 * @brief Retrieve Joint type from specified "parameterFile"
 *
 * @param parameterFile File to retrieve information from
 * @throw runtime_error if file cannot be loaded, or the file does not contain one of the elements, JointType, Properties, or ApiMap
 * @return Joint type
 *
 ******************************************************************************/
std::string JointCommandFactory::Private::getJointTypeFromParameterFile(const std::string& parameterFile)
{
        //! Parse parameter file
    TiXmlDocument file(parameterFile.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        stringstream err;
        err << "Failed to load file [" << parameterFile << "]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::INFO, "CommandFile [" + parameterFile + "] successfully loaded.");

    // Print the XML file's contents
    //cout << "Successfully loaded file: " << parameterFile << endl;
    //TiXmlPrinter printer;
    //printer.SetIndent("\t");
    //file.Accept(&printer);
    //cout << printer.Str() << endl;

    // Check for ApiMap
    TiXmlHandle parametersElement(doc.FirstChildElement("ApiMap"));
    if (parametersElement.ToElement())
    {
        // Check for Properties
        TiXmlHandle propertiesElement(parametersElement.FirstChildElement("Properties"));
        if (propertiesElement.ToElement())
        {
            // Check for JointType
            if (propertiesElement.FirstChildElement("JointType").ToElement())
            {
                return string(propertiesElement.FirstChildElement("JointType").ToElement()->Attribute("id"));
            }
            else
            {
                stringstream err;
                err << "The ApiMap file has no element named [JointType]";
                RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::ERROR, err.str());
                throw runtime_error(err.str());
            }
        }
        else
        {
            stringstream err;
            err << "The file " << parameterFile << " has no element named [Properties]";
            RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::ERROR, err.str());
            throw runtime_error(err.str());
        }
    }
    else
    {
        stringstream err;
        err << "The file " << parameterFile << " has no element named [ApiMap]";
        RCS::Logger::log("gov.nasa.robonet.JointCommandFactory", Priority::ERROR, err.str());
        throw runtime_error(err.str());
    }    

    return string();
}

