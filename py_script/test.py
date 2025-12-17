import pytest

@pytest.mark.niude


def test_requests():
    import requests , time
    from fake_useragent import UserAgent
    from datetime import datetime
    from googletrans import Translator

    ua = UserAgent()

    my_key = "175a60bc38cb90f8609bc5d8ca7c5347"
    url = f"https://api.weatherstack.com/current?access_key={my_key}&query=suzhou"
    headers = {
        "User-Agent": ua.random
    }
    response = requests.get(url, headers=headers)
    
    if response.status_code == 200:
        weather_data = response.json()
        # print(weather_data)

        location = weather_data["location"]["name"]
        temperature = weather_data["current"]["temperature"]

        now = datetime.now()
        weekday = now.weekday()
        # print(weekday)
        weather_description = weather_data["current"]["weather_descriptions"][0]
        if weather_description is not None:
            translator = Translator()
            new_result = translator.translate(weather_description, dest='zh-cn').text

        humidity = weather_data["current"]["humidity"]
        wind_speed = weather_data["current"]["wind_speed"]

        weekday_names = ["星期一", "星期二", "星期三", "星期四", "星期五", "星期六", "星期日"]

        weather_info = (
            f"\n"
            f"当前日期: {now.strftime('%Y-%m-%d')}\n"
            f"星期: {weekday_names[weekday]}\n"
            f"城市: {location}\n"
            f"温度: {temperature}°C\n"
            f"天气: {new_result}\n"
            f"湿度: {humidity}%\n"
            f"风速: {wind_speed} km/h\n"
            # f"小助手温馨提示: {niude}"
        )
        
        print(weather_info)
        
        return weather_info

if __name__ == '__main__':
    pytest.main([__file__, "-s"])
