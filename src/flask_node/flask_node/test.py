from subprocess import PIPE, run
from typing import List



# command = '''echo y | plink root@192.168.2.172 -pw easybot "{ echo "robotmode"; echo "quit"; } | nc 127.0.0.1 29999"'''
# result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
# print(result.returncode, result.stdout, result.stderr)


class RobotLowLevel:
    def __init__(self, ip: str) -> None:
        self.ip = ip

    def build_command(self, command: str) -> str:
        return f'''echo y | plink root@{self.ip} -pw easybot "{{ echo "{command}"; echo "quit"; }} | nc 127.0.0.1 29999"'''

    def execute_command(self, command: str) -> List[str]:
        result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True, shell=True)
        result = result.stdout.strip().split("\n")[1:]
        return result
    
    def get_robot_mode(self) -> str:
        command = self.build_command("robotmode")
        return self.execute_command(command)[0].replace("Robotmode: ", "")
    
    def power_on(self) -> None:
        return self.execute_command(self.build_command("power on"))
    
    def power_off(self) -> None:
        return self.execute_command(self.build_command("power off"))
    
    def shutdown(self) -> None:
        return self.execute_command(self.build_command("shutdown"))
    
    def brake_release(self) -> None:
        return self.execute_command(self.build_command("brake release"))

        
        

if __name__ == "__main__":
    robot = RobotLowLevel("192.168.2.172")
    # print(robot.execute_command(robot.build_command("robotmode")))
    print(robot.get_robot_mode())