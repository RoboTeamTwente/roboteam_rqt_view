import re
import yaml
import os.path
import rospkg

SKILL_HEADERS = rospkg.RosPack().get_path('roboteam_tactics') + "/include/roboteam_tactics/skills/"

def get_parameters(skill):
	"""Get all parameter information from a skill that is described in YAML"""

	filePath = SKILL_HEADERS + skill + '.h'

	# Check if the file exists. If it doesn't exist, it is not a skill and most likely a strategy
	if not os.path.isfile(filePath):
		return None

	# Read the header file of the skill
	with open(filePath, 'r') as skillFile:
		data = skillFile.read()

	# Regex to match comments starting with "/*" and ending with "*/"
	regex = r"(\/\*([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+\/)"
	if re.search(regex, data):
		matches = re.findall(regex, data)
		# Most of the time results in two matches. First contains the class name the other parameter info
		# If it only contains one match we assume it is only the parameter info
		if len(matches) == 1:
			match = matches[0][0]
		else:
			match = matches[1][0]
		match = match.replace("*", "").replace("/", "")

		try:
			yamlData = yaml.load(match)
		except yaml.YAMLError, exc:
			if hasattr(exc, 'problem_mark'):
				mark = exc.problem_mark
				print "YAML parse error in file " + filePath + ". Position: (%s:%s)" % (mark.line+1, mark.column+1)
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


def get_skill_descriptions(skills):
	"""Get descriptions from skills as described in the YAML in their header files"""

	# Result will be a list of (skill, description) tuples
	result = []
	for i in range(len(skills)):
		filePath = SKILL_HEADERS + skills[i] + '.h'

		# Check if the file exists. If it doesn't exist, it is not a skill and most likely a strategy.
		# As of now, strategies do not have descriptions
		if not os.path.isfile(filePath):
			result.append((skills[i], None))
			continue

		# Read the header file of the skill
		with open(filePath, 'r') as skillFile:
			data = skillFile.read()

		# Regex to match comments starting with "/*" and ending with "*/"
		regex = r"(\/\*([^*]|[\r\n]|(\*+([^*/]|[\r\n])))*\*+\/)"
		if re.search(regex, data):
			matches = re.findall(regex, data)
			# Most of the time results in two matches. First contains the class name the other parameter info
			# If it only contains one match we assume it is only the parameter info
			if len(matches) == 1:
				match = matches[0][0]
			else:
				match = matches[1][0]
			match = match.replace("*", "").replace("/", "")
			try:
				yamlData = yaml.load(match)
			except yaml.YAMLError, exc:
				if hasattr(exc, 'problem_mark'):
					mark = exc.problem_mark
					print "YAML parse error in file " + filePath + ". Position: (%s:%s)" % (mark.line+1, mark.column+1)
				result.append((skills[i], None))
				continue
			if 'Descr' not in yamlData:
				result.append((skills[i], None))
			else:
				result.append((skills[i], yamlData['Descr']))
		else:
			# No YAML
			result.append((skills[i], None))
	return result

