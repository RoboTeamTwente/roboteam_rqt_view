import re
import yaml
import os.path
import rospkg
import sys
import json

SKILL_HEADERS = rospkg.RosPack().get_path('roboteam_tactics') + "/include/roboteam_tactics/skills/"
SKILLS = rospkg.RosPack().get_path('roboteam_tactics') + "/src/skills/"
TREE_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/json/"
PROJECTS_DIR = rospkg.RosPack().get_path('roboteam_tactics') + "/src/trees/projects/"


def get_parameters(skill):
    """Get all parameter information from a skill that is described in YAML"""

    # ---- Read file ----

    filePath = SKILL_HEADERS + skill + '.h'
    # Check if the file exists. If it doesn't exist, it is not a skill and most likely a strategy
    if not os.path.isfile(filePath):
    	return None
    # Read the header file of the skill
    with open(filePath, 'r') as skillFile:
    	data = skillFile.read()

    # ---- /Read file ----


    # ---- YAML ----

    # Regex to match comments starting with "/*" and ending with "*/"
    # More information at: https://blog.ostermiller.org/find-comment
    regex = r"(\/\*([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+\/)"
    if re.search(regex, data):
    	matches = re.findall(regex, data)

    	# Most of the time results in two matches. First contains the class name the other parameter info
    	# If it only contains one match we assume it is only the parameter info
    	if len(matches) == 1:
    		match = matches[0][0]
    	else:
    		match = matches[1][0]

        # Replace comment tokens
    	match = match.replace("*", "").replace("/", "")

        # Parse YAML
    	try:
    		yamlData = yaml.load(match)
    	except yaml.YAMLError, exc:
    		if hasattr(exc, 'problem_mark'):
    			mark = exc.problem_mark
    			print >> sys.stderr, "YAML parse error in file " + filePath + ". Position: (%s:%s)" % (mark.line+1, mark.column+1)
    		return None
    	if 'Params' not in yamlData:
    		return None
    	# Convert array to dictionary
    	parameters = {}
    	for param in yamlData['Params']:
    		# Filter out ROBOT_ID, since it is already defined elsewhere in the gui
    		if param.keys()[0] != "ROBOT_ID":
    			parameters[param.keys()[0]] = param.get(param.keys()[0])
    	return parameters if len(parameters) > 0 else None
    else:
    	# No YAML. Assuming that no parameters can be set
    	return None

    # ---- /YAML ----


def get_skill_descriptions(skills):
    """Get descriptions from skills as described in the YAML in their header files"""

    # Result will be a list of (skill, description) tuples
    result = []
    for i in range(len(skills)):

    	# ---- Read files ----

    	filePath = SKILL_HEADERS + skills[i] + '.h'
    	# Check if the file exists. If it doesn't exist, it is not a skill and most likely a strategy.
    	# As of now, strategies do not have descriptions
    	if not os.path.isfile(filePath):
    		result.append((skills[i], None))
    		continue
    	# Read the header file of the skill
    	with open(filePath, 'r') as skillFile:
    		data = skillFile.read()

    	# ---- /Read files ----


    	# ---- YAML ----

    	# Regex to match comments starting with "/*" and ending with "*/"
        # More information at: https://blog.ostermiller.org/find-comment
    	regex = r"(\/\*([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+\/)"
    	if re.search(regex, data):
            matches = re.findall(regex, data)

            # Most of the time results in two matches. First contains the class name the other parameter info
            # If it only contains one match we assume it is only the parameter info
            if len(matches) == 1:
                match = matches[0][0]
            else:
                match = matches[1][0]

            # Remove comment tokens
    		match = match.replace("*", "").replace("/", "")

            # Description when no description is available
            defaultDescription = "No description available"

            # Parse YAML
            try:
                yamlData = yaml.load(match)
            except yaml.YAMLError, exc:
                if hasattr(exc, 'problem_mark'):
                    mark = exc.problem_mark
                    print >> sys.stderr, "YAML parse error in file " + filePath + ". Position: (%s:%s)" % (mark.line+1, mark.column+1)
                result.append((skills[i], defaultDescription))
                continue
            if 'Descr' not in yamlData:
                result.append((skills[i], defaultDescription))
            else:
                result.append((skills[i], yamlData['Descr']))
        else:
            # No YAML
            result.append((skills[i], defaultDescription))

        # ---- /YAML ----
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
    for file_name in os.listdir(TREE_DIR):
        if file_name.endswith(".json"):
            strategies.append(file_name[:-5])
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

