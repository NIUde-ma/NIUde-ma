#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

try:
    import os , subprocess ,re ,sys
    import common , requests , time
    from datetime import datetime, timedelta
except ImportError as e:
    common.print_red(f"not found {e}")


def command_shell_script(shell_path="./GET_COF.sh"):
    try:
        command_cmd = f"bash {shell_path}"

        result = subprocess.run(
            command_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=20
        )

        if result.returncode == 0:
            output = result.stdout.decode('utf-8').strip()
            # print(output)
            return 0 , output
        else:
            # print(result)
            return 1 , f"vehicle config get false"

    except subprocess.TimeoutExpired:
        common.print_red("cmd command is timeout")
    except Exception as e:
        common.print_red(f"cmd command is errors: {e}")  

# if __name__ == "__main__":
#     command_shell_script(shell_path="./GET_COF.sh")