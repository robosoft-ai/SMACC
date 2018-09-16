#include "nasa_robodyn_mechanisms_core/JointControlManagerFactory.h"
/***************************************************************************//**
 *
 * @brief Generate new joint control manager factory
 *
 * @param mechanism
 * @param io
 * @param timeLimit
 * @param nodeRegisterManager
 * @throw runtime_error if an unsupported joint type is found in the ControlFile
 * @return The new control manager factory
 *
 ******************************************************************************/
JointControlManagerPtr JointControlManagerFactory::generate(const std::string& mechanism, const JointControlCommonInterface::IoFunctions& io, double timeLimit, NodeRegisterManagerPtr nodeRegisterManager)
{
    JointControlManagerPtr newManager;
    std::string            registerFile;
    std::string            jointType;

    registerFile = io.getRegisterFile(mechanism);
    if(registerFile.empty())
    {
        RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::ERROR, "No control file specified for mechanism " + mechanism);
        newManager.reset();
        return newManager;
    }

    jointType = Private::getPropertyFromFile(registerFile, "NodeType");

    // Simple types
    if (jointType == "SeriesElastic")
    {
        nodeRegisterManager->addNode(mechanism, registerFile);
        newManager = boost::make_shared<JointControlManagerSeriesElastic>(mechanism, io, timeLimit, jointType, nodeRegisterManager);
    }
    else if (jointType == "Rigid")
    {
        nodeRegisterManager->addNode(mechanism, registerFile);
        newManager = boost::make_shared<JointControlManagerSeriesElastic>(mechanism, io, timeLimit, jointType, nodeRegisterManager);
    }
    else if (jointType == "Gripper")
    {
        nodeRegisterManager->addNode(mechanism, registerFile);
        newManager = boost::make_shared<JointControlManagerGripper>(mechanism, io, timeLimit, jointType, nodeRegisterManager);
    }

    // Complex types
    else if (jointType == "Wrist")
    {
        newManager = boost::make_shared<JointControlManagerWrist>(mechanism, io, timeLimit, jointType);
    }
    else if (jointType == "Thumb")
    {
        newManager = boost::make_shared<JointControlManagerFinger>(mechanism, io, timeLimit, jointType);
    }
    else if (jointType == "PrimaryFinger")
    {
        newManager = boost::make_shared<JointControlManagerFinger>(mechanism, io, timeLimit, jointType);
    }
    else if (jointType == "SecondaryFingers")
    {
        newManager = boost::make_shared<JointControlManagerFinger>(mechanism, io, timeLimit, jointType);
    }
    else
    {
        std::stringstream err;
        err << "Unsupported joint type [" << jointType << "] found in ControlFile";
        RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::ERROR, err.str());
        throw std::runtime_error(err.str());
    }

    return newManager;
}
/***************************************************************************//**
 *
 * @brief Retrieve property of element from the given file
 * @param filename File to get information from
 * @param property Element property to retrieve
 * @throw runtime_error if file failed to load
 * @throw missingXMLElementException if file is missing a specified element or property
 *
 ******************************************************************************/
std::string JointControlManagerFactory::Private::getPropertyFromFile(const std::string& filename, const std::string& property)
{
    TiXmlDocument file(filename.c_str());
    bool loadOkay = file.LoadFile();
    if (!loadOkay)
    {
        std::stringstream err;
        err << "Failed to load file [" << filename << "]";
        RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::FATAL, err.str());
        throw std::runtime_error(err.str());
    }
    TiXmlHandle doc(&file);
    RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::INFO, "Register file [" + filename + "] successfully loaded.");
    RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::INFO, "XML being parsed for [RegisterData]");

    // Check for RegisterData
    if (!(doc.FirstChildElement("RegisterData").ToElement()))
    {
        RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::ERROR, "The file " + filename + " has no element named [RegisterData]");
        MissingXMLElementException missingXMLElementException;
        throw missingXMLElementException;
    }

    // Check for Properties
    std::stringstream ss;
    TiXmlHandle registersElement(doc.FirstChildElement("RegisterData").FirstChildElement("Properties"));
    if (registersElement.ToElement())
    {
        // Check for property
        if (!(registersElement.FirstChildElement(property).ToElement()))
        {
            RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::ERROR, "The file " + filename + " has no property named [" + property + "]");
            MissingXMLElementException missingXMLElementException;
            throw missingXMLElementException;
        }

        ss << registersElement.FirstChildElement(property).ToElement()->GetText();
        return ss.str();
    }
    else
    {
        RCS::Logger::log("gov.nasa.robonet.JointControlManagerFactory", log4cpp::Priority::ERROR, "The file " + filename + " has no element named [Properties]");
        MissingXMLElementException missingXMLElementException;
        throw missingXMLElementException;
    }
}
