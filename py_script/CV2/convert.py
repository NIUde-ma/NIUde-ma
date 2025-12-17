import math
import json

def get_json_keys():
    with open("./LDR_calib.json",'r',encoding='utf-8') as f:
        data = json.load(f)
        roll = data.get("rolls", "Unknown")
        yaw = data.get("yaws", "Unknown")
        pitch = data.get("pitchs", "Unknown")
        car_id = data.get("car_ids", "Unknown")

    return roll , yaw , pitch , car_id

def main(roll , yaw , pitch , car_id):
    
    result = {
    }

    roll_deg = roll * 180 / math.pi
    pitch_deg = pitch * 180 / math.pi
    yaw_deg = yaw * 180 / math.pi
    
    result[f'{car_id}-roll'] = str(abs(roll_deg))[0:4]
    result[f'{car_id}-pitch'] = str(abs(pitch_deg))[0:4]
    result[f'{car_id}-yaw'] = str(abs(yaw_deg))[0:4]

    print(result)

if __name__ == '__main__':
    roll , yaw , pitch , car_id = get_json_keys()
    main(roll , yaw , pitch , car_id)