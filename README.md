# GuideBot
Дипломный проект в МГТУ им. Баумана

**[Switch to English](README_ENG.md)**

# Описание #

# Материалы #
- [Облако](https://drive.google.com/drive/folders/1_WAW31vtC9gaj0y2e-LKjm6cnkcJSpfA)
- [Notion](https://www.notion.so/Robo-f3258db8742045b69fb7596a01b7c649)
- [Ведомость](https://docs.google.com/spreadsheets/d/1mbreOyS2strwakkSfiQ9_aJJ-Iqti-MNq-A9kCi9TpU/edit#gid=0)

# Научрук #
- [Антон Канев](http://iu5.bmstu.ru/user/profile.php?id=2271)

# Разработчики #
- [Кравцов Андрей](https://vk.com/kravtandr)
- [Волгина Анна](https://vk.com/luckyhorseshoe)
- [Кириллов Денис](https://vk.com/denactive)
- [Сысойкин Егор](https://bmstu.codes/e.sysoykin)
- [Игорь Шпак](https://bmstu.codes/i.shpak)
- [Дьяконова Светлана](#)

# Установка и начало работы #

### Разработка
**dev-сервер + Django**

**Запуск (из корня проекта)**

Скрипт пока только под Windows 10
  1. ```npm i```
  2. ```npm start```

Может, сделаю и для Linux или кросс-платформенный скрипт с TMUX.
- Ubuntu 20.04
  1. Открыть 2 терминала.
  2. См разделы **Установка** секций **Сервер** и **Web-клиент**.


### Сервер
**Технические требования**

- Windows 10 / Ubuntu 20.04

**Программное обеспечение**

- python 3.9
- python libraries from [requirements.txt](./ServerPy/requirements.txt)

**Установка**

1. ```cd ServerPy```
2. ```sudo apt-get install python3.9```
3. __Необязательно__ В Linux интерпретатор питона именуется __python3__, а в  Windows - __py__.

    ```alias py=python3```
4. __Необязательно__ Этот шаг можно пропустить, если не хотите использовать virtual environment. Переходите на пункт 6.

    ```sudo apt-get install python3.9-venv```
5. ```py -m venv venv```
6. Windows: ```./venv/Scripts/activate```
    
    Linux: ```source venv/bin/activate```
7. В командной строке может появиться красное сообщение про django-logger. Это __нормально__.

    ```pip install -r requirements```
8. ```py manage.py collectstatic``` 
9. ```py manage.py runserver``` 

---

### Веб-клиент
**Технические требования**

- Windows 10 / Ubuntu 20.04

**Программное обеспечение**

- npm 6.14.11+
- node 14.16.0+
- js libraries from [package.json](./WebClient/package.json)

**Установка**

1. ```cd WebClient```
2. ```sudo apt-get install nodejs npm```
3. ```npm install -g npm@latest```
4. ```npm i```
5. ```npm start```

---

### Мобильное приложение
**Технические требования**

- Android 8.0+

**Программное обеспечение**

-

**Установка**

-

---

### Мобильная платформа PI
**Технические требования**

- Raspberry PI 4

**Программное обеспечение**

-

**Установка**

-


# Лицензия #
[Лицензия GNU GPLv2](./LICENSE)
