<img width=100% src="https://capsule-render.vercel.app/api?type=waving&color=00bfbf&height=120&section=header"/>
<h1 align="center"><img align="center" style="border-radius: 100%;" src="https://moodle.embarcatech.cepedi.org.br/pluginfile.php/1/theme_moove/logo/1733422525/Group%20658.png" width="300px;" alt=""><br><br>BitDogLab</h1>

<h3 align="center">
    Controlador de LEDs e Periféricos para a placa BitDogLab
</h3>

---

Tabela de conteúdos
=================
<!--ts-->
   * [Sobre o projeto](#-sobre-o-projeto)
   * [Layout Repositorio Github](#-layout-repositorio-github)
   * [Funcionalidades](#-Funcionalidades)
   * [Como executar o projeto](#-como-executar-o-projeto)
   * [Imagens do Projeto](#-imagens-do-projeto)
     * [Imagem](#-projeto-na-extensão-wokwi-simulator-no-visual-studio)
     * [Vídeo](#-video-do-projeto)
   * [Tecnologias](#-tecnologias)
     * [Websites](#-websites)
     * [Utilitários](#user-content-server--nodejs----typescript)
   * [Desenvolvedor](#-desenvolvedor)
   * [Licença](#-licença)
<!--te-->

---

## 💻 Sobre o projeto

O projeto BitDogLab é um sistema embarcado desenvolvido para a placa BitDogLab, com funcionalidades voltadas ao controle e monitoramento de periféricos, incluindo um display OLED, LEDs, botões e comunicação via I2C.

Este projeto justifica-se por ser uma solução didática e acessível para aprendizado de sistemas embarcados, permitindo aos estudantes explorar conceitos fundamentais de eletrônica e programação.

Projeto desenvolvido durante o curso de Formação Básica em Software Embarcado oferecido pela [Embarcatech](https://embarcatech.softex.br).
A Formação Básica em Software Embarcado da Embarcatech é um programa de capacitação profissional técnica destinado a alunos de nível superior e técnico em Tecnologias da Informação e Comunicação (TIC) e áreas correlatas, focado em tecnologias de Sistemas Embarcados.

---

## 🎨 Layout Repositorio Github
<i>

<h3>BitDogLab/</h3>

- .vscode
  - c_cpp_properties.json

  - cmake-kits.json

  - extensions.json

  - launch.json

  - settings.json

  - tasks.json

- assets/

- CMakeLists.txt

- LICENSE

- README.md

- bitdoglab.c

- diagram.json

- font.h

- pico_sdk_import.cmake

- ssd1306.c

- ssd1306.h

- wokwi.toml

- ws2818b.pio

- ws2818b.pio.h


</i>

---

## ⚙️ Funcionalidades
- Microcontrolador: Responsável pelo processamento e controle geral.
- Display OLED: Exibição de informações para o usuário.
- NeoPixels: Iluminação RGB controlada via protocolo próprio.
- Buzzer: Indicação sonora de eventos.
- Sensores: Captura de informações do ambiente.

---

## 🚀 Como executar o projeto

💡Siga as instruções abaixo para configurar, compilar e executar o programa.

### Pré-requisitos

Antes de começar, você vai precisar ter instalado em sua máquina as seguintes ferramentas:
  - Sistema operacional Linux, macOS ou Windows (com suporte a Makefile).
  - [Git](https://git-scm.com) (Opcional, mas recomendado),
  - [GCC compilador](https://gcc.gnu.org)
  - [Biblioteca Pico-Sdk](https://github.com/raspberrypi/pico-sdk.git) (OBS: Necessário caso queira modificar o projeto)

Além disto é bom ter um editor para trabalhar com o código como [VSCode](https://code.visualstudio.com/) com a extensão [Raspberry](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)  e usar o simulador web [Wokwi](https://wokwi.com) (ou a extensão do Vscode [Wokwi Simulator](https://marketplace.visualstudio.com/items?itemName=Wokwi.wokwi-vscode))

### 🎲 Dowload do Projeto

#### Dowload do Projeto no Desktop
- Opção 1:
  - Abra o terminal de comando Git Bash 
  - Clone o repositório do GitHub com o comando:
```
$ git clone https://github.com/Erlon335/BitDogLab.git
```
- Opção 2:
  - No repósitorio [BitDogLab](https://github.com/Erlon335/BitDogLab) aperte o Botão <i><>code</i>
  - Aperte a opção <i>Dowload ZIP</i>


### 🎲 Rodando a Animação no Wokwi

#### Wokwi Web
- Entre no navegador e digite [Wokwi.com]()
- Faça Upload do projeto <i>diagram.json</i>
- Faça upload dos aquivos <i>bitdoglab.c</i>, <i>ssd1306.c</i>, <i>ssd1306.h</i>, <i>font.h</i> e <i>ws2818b.pio.h</i>

#### Extensão Wokwi
- Abra o Visual Studio
- Na aba da extensão [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico), aperte para Importar o projeto
- Compile o projeto
- crie um arquivo (caso não tenha no projeto) <i>wokwi.toml</i> e digite o código:
```
[wokwi]
version = 1
firmware = 'build/bitdoglab.hex'
elf = 'build/bitdoglab.elf'
```
- Abra o arquivo <i>diagram.json</i>


### 🎲 Rodando as Animações na placa BitdogLab

#### Placa BitDogLab
- Através de um cabo USB conecte a placa ao seu Disposito
- Aperte o Botão Bootsel e Reset 

#### VsCode Studio
- Abra o Visual Studio
- Na aba da extensão [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico), aperte para Importar o projeto
- Compile o projeto
- Entre na pasta <i>build/</i>
- Cole o arquivo <i>bitdoglab.uf2</i> no armazenamento placa BitDog
<br>


---

## 🎥 Imagens do Projeto

### 💿 Diagrama Visual do Projeto no Wokwi
<p align="center"><img width="700" height="400" src="https://github.com/Erlon335/BitDogLab/blob/main/assets/Diagrama%20BitDogLab.png"></p>

### 💿 Video Demonstrativo do Projeto na Placa






<p align="center">https://github.com/user-attachments/assets/d81fbe14-d662-4b3b-a383-f226d2107ec8</p>






- link para Dowload: (https://drive.google.com/file/d/1HTZtXd0qhRAw7qP8tzSLDZZ7R78vbDt4/view?usp=sharing)

---

## 🛠 Tecnologias

As seguintes ferramentas foram usadas na construção do projeto:

#### **Websites**
-   **[Visual Studio code](https://code.visualstudio.com)**
-   **[Github](https://github.com)**
-   **[Wokwi Web](https://gcc.gnu.org)**


#### **Utilitários**

-   Editor:  **[Visual Studio Code](https://code.visualstudio.com/)**  → Extensions:  **[C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools), [C/C++ Compile Run](https://marketplace.visualstudio.com/items?itemName=danielpinto8zz6.c-cpp-compile-run), [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) e [Wokwi Simulator](https://marketplace.visualstudio.com/items?itemName=Wokwi.wokwi-vscode)**
-   **[Git](https://git-scm.com)**


---

## 👨‍💻 Desenvolvedor

GRUPO 1, SUBGRUPO 3 da Embarcatech <br/>
Mentor: MANOEL MESSIAS DA SILVA JUNIOR

<table>
  <tr>
    <td align="center"><img style="border-radius: 50%;" src="https://avatars.githubusercontent.com/u/180613216?v=4" width="100px;"/><br/><a href="https://github.com/Erlon335">Érlon S. Alves Neto<a/><br/><br/></td>
</table>
      
---

## 📄 Licença

Este projeto está sob a licença de Érlon Alves da Formação Básica em Software Embarcado da Embarcatech - Veja o arquivo <a href="https://github.com/Erlon335/BitDogLab/blob/main/LICENSE">License.md<a/>

<img width=100% src="https://capsule-render.vercel.app/api?type=waving&color=00bfbf&height=120&section=footer"/>
