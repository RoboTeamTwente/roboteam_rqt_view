import re
import yaml
import os.path
import rospkg
import sys
import json

SKILL_HEADERS = rospkg.RosPack().get_path('roboteam_tactics') + "/include/roboteam_tactics/skills/"
SKILLS = rospkg.RosPack().get_path('roboteam_tactics') + "/src/skills/"
PROJECTS_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/projects/"

# Parameter that indicates whether functions of this class should give feedback on improper formatted YAML
YAML_FEEDBACK = True


def get_parameters(skill):
    """Get all parameter information from a skill that is described in YAML"""
    # Construct file path
    filePath = SKILL_HEADERS + skill + '.h'
    # Read the contents of the file
    contents = get_file_contents(filePath)
    # If there was an error reading the file, return None
    if contents is None:
        return None
    # Scavenge YAML
    yamlText = get_yaml_from_file(filePath, contents)
    # If YAML wasn't scavenged correctly, return None
    if yamlText is None:
        return None
    # Parse YAML
    data = yaml_validator(filePath, yamlText)
    if data is None or 'Params' not in data:
        return None
    
    # Convert array to dictionary to preserve all information
    parameters = {}
    for param in data['Params']:
        # Filter out ROBOT_ID, since it is already defined elsewhere in the gui
        if param.keys()[0] != "ROBOT_ID":
            parameters[param.keys()[0]] = param.get(param.keys()[0])
            
    return parameters if len(parameters) > 0 else None


def get_skill_descriptions(skills):
    """Get descriptions from skills as described in the YAML in their header files"""

    # Result will be a list of (skill, description) tuples
    result = []
    for i in range(len(skills)):
        # Ignore the unity.h file, as it is not a skill
        if skills[i] == 'unity':
            continue
        # Construct file path
        filePath = SKILL_HEADERS + skills[i] + '.h'
        # Read the contents of the file
        contents = get_file_contents(filePath)
        # If there was an error reading the file, forward to next iteration
        if contents is None:
            result.append((skills[i], defaultDescription))
            continue
        # Scavenge YAML
        yamlText = get_yaml_from_file(filePath, contents)
        # If YAML wasn't scavenged correctly, forward to next iteration
        if yamlText is None:
            result.append((skills[i], defaultDescription))
            continue
        # Parse YAML
    	data = yaml_validator(filePath, yamlText)
        # Description when no description is available
        defaultDescription = "No description available"
    	# If YAML wasn't parsed correctly, forward to next iteration
    	if data is None:
            result.append((skills[i], defaultDescription))
    	    continue
        description = data['Descr']
        if description is None:
            description = defaultDescription
        result.append((skills[i], description))
    return result


def get_strategy_descriptions(strategies):
    result = []
    defaultDescription = "No description available"
    # Strategies do not have descriptions
    for i in range(len(strategies)):
        result.append((strategies[i], defaultDescription))
    return result


def get_information():
    """Gets the names and descriptions of all skills and strategies"""

    # ---- Skills ----

    # Read the names of every file in the skills folder
    skills = []
    fileNames = os.listdir(SKILLS)
    # Add those with a .cpp extension and remove that extension in the code
    for fileName in fileNames:
        if fileName.endswith(".cpp"):
    	    skills.append(fileName[:-4])
    # Sort skills alphabetically
    skills.sort()

    # ---- /Skills ----


    # ---- Strategies ----

    # Find all strategies in the trees
    strategies = []
    for file_name in os.listdir(PROJECTS_DIR):
        if file_name.endswith(".b3"):
            with open(PROJECTS_DIR + file_name) as data_file:
                data = json.load(data_file)['data']
            if 'trees' in data:
                for tree_data in data['trees']:
                    strategies.append(file_name[:-3] + "/" + tree_data['title'])
    # Sort strategies alphabetically
    strategies.sort()

    # ---- /Strategies ----


    # ---- Descriptions ----

    skills = get_skill_descriptions(skills)
    strategies = get_strategy_descriptions(strategies)

    # ---- /Description ----

    return (skills, strategies)


def get_file_contents(filePath):
    # Check if the file exists. If it doesn't exist, it is not a skill and most likely a strategy.
    # As of now, strategies do not have descriptions
    if not os.path.isfile(filePath):
        print_feedback(filePath, "File not found.")
        return None
    # Read the header file of the skill
    with open(filePath, 'r') as f:
        data = f.read()
    return data


def get_yaml_from_file(filePath, contents):
    # Regex to match all comments starting with "/*" and ending with "*/"
    # More information at: https://blog.ostermiller.org/find-comment
    regex = r"(\/\*([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+\/)"
    if re.search(regex, contents):
        matches = re.findall(regex, contents)
        # Length should be at least 2, since the YAML should specify the class in a comment and then the description and parameters
        if len(matches) < 2:
            print_feedback("YAML parse error in file " + filePath + ". Did you specify the class?")
            return None
        match = matches[1][0]
        
        # Remove comment tokens
        yamlText = match.replace("*", "").replace("/", "")
    else:
        print_feedback(filePath, "No YAML found.")
        return None
        
    return yamlText


def yaml_validator(filePath, yamlText):
    """Validates YAML and returns object constructed out of the YAML data or None if the data is incorrect.
       Validates specifically for how we want skill header files to be formatted."""
    
    try:
        yamlData = yaml.load(yamlText)
    except yaml.YAMLError, exc:
        if hasattr(exc, 'problem_mark'):
            mark = exc.problem_mark
            print_feedback(filePath, "Position: (%s:%s)" % (mark.line+1, mark.column+1))
        else:
            print_feedback(filePath, "")
        return None
    
    # Check if the YAML is expected
    if 'Descr' not in yamlData:
        print_feedback(filePath, "No description specified.")
        return None
    elif yamlData['Descr'] is None:
        print_feedback(filePath, "Description is empty.")

    if 'Params' in yamlData:
        # If yamlData['Params'] returns a dict instead of a list the YAML was incorrectly formatted
        # Parameters should be listed using a '-'
        if isinstance(yamlData['Params'], dict):
            print_feedback(filePath, "Parameters incorrectly formatted. Should be listed using \'-\'.")
            return None
            
        for param in yamlData['Params']:
            arguments = param.get(param.keys()[0])
            parameterName = param.keys()[0]
            # Check if the parameter has a valid Type
            if 'Type' not in arguments:
                print_feedback(filePath, parameterName + " is missing its Type.")
                return None
            t = arguments['Type']
            if t != "String" and t != "Double" and t != "Int" and t != "Bool":
                print_feedback(filePath, parameterName + " has an invalid Type.")
                return None
            
            # Check against an empty description
            if 'Descr' in arguments and arguments['Descr'] is None:
                print_feedback(filePath, parameterName + " has an empty description.")
                return None
            
            # Check against an empty default value
            if 'Default' in arguments and arguments['Default'] is None:
                print_feedback(filePath, parameterName + " has an empty default value.")
                return None
                
            if 'Can be' in arguments:
                # If yamlData['Params']['Can be'] returns a dict instead of a list the YAML was incorrectly formatted.
                # Options should be listed using a '-'
                if isinstance(arguments['Can be'], dict):
                    print_feedback(filePath, "Options in \'Can be\' in parameter " + parameterName + " incorrectly formatted. Options should be listed using \'-\'.")
                    return None
    return yamlData


def print_feedback(filePath, message):
    if YAML_FEEDBACK:
        print >> sys.stderr, "YAML parse error in file " + filePath + ". " + message
