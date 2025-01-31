бек: https://github.com/kravtandr/Bbot-image-bucket

В конфиге прописать айпи для бека и токен для доступа к апи LLM
В коде найти все 193.XXX.XX.XX и заменить

imgur там резервыный вариант, он за 1 прогон выбивает лимит по апи

roslaunch abot_description sim.launch
python3 ./Scripts/llm_brain/main.py