import os
from subprocess import run
import json
from pathlib import Path

class Processes:
    def __init__(self):
        self.message_dict = {}
        self.action_dict = {}
        self.service_dict = {}

        self.message_libs = {}
        self.action_libs = {}
        self.service_libs = {}
        self.ros2Primitives = {
            # Boolean
            'bool': False,
            
            # Byte types
            'byte': 0,
            'char': 0,
            
            # Floating point
            'float32': 0.0,
            'float64': 0.0,
            
            # Signed integers
            'int8': 0,
            'int16': 0,
            'int32': 0,
            'int64': 0,
            
            # Unsigned integers
            'uint8': 0,
            'uint16': 0,
            'uint32': 0,
            'uint64': 0,
            
            # Strings
            'string': '',
            'wstring': ''
        }



    def assign_libs_and_count(self):
        for line in self.message_dict:
            lib = line.split("/")[0]
            self.message_libs[lib] = 0

        for line in self.action_dict:
            lib = line.split("/")[0]
            self.action_libs[lib] = 0

        for line in self.service_dict:
            lib = line.split("/")[0]
            self.service_libs[lib] = 0

            

    def process_msgs(self):
        output = run(["ros2", "interface", "list"], capture_output=True, text=True, check=True)
        output_str = output.stdout
        output_line_split = output_str.split("\n")

        add_message = False
        add_action = False
        add_service = False

        for line in output_line_split:
            if line == "Messages:":
                add_message = True
                add_action = False
                add_service = False
                continue

            elif line == "Actions:":
                add_message = False
                add_action = True
                add_service = False
                continue

            elif line == "Services:":
                add_message = False
                add_action = False
                add_service = True
                continue

            if add_message:
                self.message_dict[line[4:]] = 0 # remove leading spaces

            elif add_action:
                self.action_dict[line[4:]] = 0

            elif add_service:
                self.service_dict[line[4:]] = 0



    def parse_message_structure(self, lines, start_idx=0, indent_level=0):
        result = {}
        i = start_idx
        
        while i < len(lines):
            line = lines[i]
            
            if not line or line.strip()[0] == "#":
                i += 1
                continue
            
            current_indent = len(line) - len(line.lstrip('\t'))
            
            if current_indent < indent_level:
                return result, i
            
            if current_indent > indent_level:
                i += 1
                continue
            
            parts = line.strip().split()
            if len(parts) < 2:
                i += 1
                continue
                
            data_type = parts[0]
            data_label = parts[1]
            
            if self.is_primitive_type(data_type):
                result[data_label] = self.get_default_value(data_type)
                i += 1
            else:
                nested_data, new_idx = self.parse_message_structure(
                    lines, i + 1, indent_level + 1
                )
                result[data_label] = nested_data
                i = new_idx
        
        return result, i


    def parse_service_structure(self, lines):
        
        if '---' not in lines:
            return {}, {}

        sep_index = lines.index('---')

        request_lines = lines[:sep_index]
        response_lines = lines[sep_index + 1:]

        request_data, _ = self.parse_message_structure(request_lines)
        response_data, _ = self.parse_message_structure(response_lines)

        return request_data, response_data

    def parse_action_structure(self, lines):
        
        if '---' not in lines:
            return {}, {}, {}
        
        separators = []
        for i, line in enumerate(lines):
            if line.strip() == '---':
                separators.append(i)
        
        if len(separators) < 2:
            return {}, {}, {}
        
        first_separator = separators[0]
        second_separator = separators[1]
        
        goal_lines = lines[:first_separator]
        result_lines = lines[first_separator + 1:second_separator]
        feedback_lines = lines[second_separator + 1:]
        
        goal_msg, _ = self.parse_message_structure(goal_lines)
        result_msg, _ = self.parse_message_structure(result_lines)
        feedback_msg, _ = self.parse_message_structure(feedback_lines)
        
        return goal_msg, result_msg, feedback_msg




    def convert_to_json(self,save_dir):
        for msg in self.message_dict:
            if self.message_libs[msg.split("/")[0]]:
                output = run(["ros2", "interface", "show", msg], 
                            capture_output=True, text=True, check=True)
                lines = output.stdout.strip().split("\n")
                
                line_data, _ = self.parse_message_structure(lines)
                
                ros_json = {
                    "topic": "",
                    "msgType": msg,
                    "data": line_data
                }


                write_directory = Path(save_dir) / msg.split("/")[0] / msg.split("/")[1]
                write_directory.mkdir(parents=True, exist_ok=True)

                file_name = write_directory / f"{msg.split('/')[2]}.json"

                with open(file_name, "w") as json_file:
                    json.dump(ros_json, json_file, indent=4)
                print(json.dumps(ros_json, indent=4))


        for srv in self.service_dict:
            if not self.service_libs[srv.split("/")[0]]:
                continue

            output = run(
                ["ros2", "interface", "show", srv],
                capture_output=True,
                text=True,
                check=True
            )

            lines = output.stdout.strip().split("\n")

            request_data, response_data = self.parse_service_structure(lines)

            ros_json = {
                "service": "",
                "srvType": srv,
                "request": request_data,
                "response": response_data
            }

            write_directory = (
                Path(save_dir) /
                srv.split("/")[0] /
                srv.split("/")[1]
            )
            write_directory.mkdir(parents=True, exist_ok=True)

            file_name = write_directory / f"{srv.split('/')[2]}.json"

            with open(file_name, "w") as json_file:
                json.dump(ros_json, json_file, indent=4)

            print(json.dumps(ros_json, indent=4))

        for act in self.action_dict:
            if not self.action_libs[act.split("/")[0]]:
                continue

            output = run(
                ["ros2", "interface", "show", act],
                capture_output=True,
                text=True,
                check=True
            )

            lines = output.stdout.strip().split("\n")

            goal_msg, result_msg, feedback_msg = self.parse_action_structure(lines)

            ros_json = {
                "action": "",
                "actionType": srv,
                "goal": goal_msg,
                "result": result_msg,
                "feedback": feedback_msg
            }

            write_directory = (
                Path(save_dir) /
                act.split("/")[0] /
                act.split("/")[1]
            )
            write_directory.mkdir(parents=True, exist_ok=True)

            file_name = write_directory / f"{act.split('/')[2]}.json"

            with open(file_name, "w") as json_file:
                json.dump(ros_json, json_file, indent=4)

            print(json.dumps(ros_json, indent=4))




    def is_primitive_type(self, type_string):
    
        base_type = type_string.split('[')[0]
        
        return base_type in self.ros2Primitives



    def get_default_value(self, type_string):
        base_type = type_string.split('[')[0]
        
        if base_type in self.ros2Primitives:
            # It's an array type
            if '[' in type_string:
                return []
            # It's a base type
            return self.ros2Primitives[base_type]
        
        return None


#Preprocess here (gui)




"""
for msg in message_dict:
    output = run(["ros2", "interface", "show", msg], capture_output=True, text=True, check=True)
    output_str = output.stdout
    #print(output_str)
"""
    


#Assumes input of type library/type/name
