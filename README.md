# SERR-01

Aqui contem as bibliotecas e o codigo principal do firmware desenvolvido para o SERR-01 como projeto de conclusão de curso em engenharia da computação na Fainor - Faculdade Independente do Nordeste.

Obs.: Foi utilizada a biblioteca motor_control de , entretanto foi modificada para funcionar com a Ponte H IBT-2 juntamente com o SERR-01. 
link do repositorio: https://github.com/1988kramer/motor_control 

O sistema se baseia em troca de mensagens entre um Arduino Mega 2560 e um Raspberry Py 3 através do ROS e a biblioteca rosserial.

Por Exemplo, para a plataforma robótica móvel se mover um metro para frente, é executado o comando a seguir no Raspberry:
  
  Abra um terminal e execute o codigo a seguir para iniciar a comunicação entre o Arduino e o Raspberry:
    rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=57600;
    
  Abra um segundo terminal para enviar a mensagem, e execute o comando: 
    rostopic pub -1 mover std_msgs/Float64 1000
    
    
Lista de Comandos:
Andar um metro a frente
```
rostopic pub -1 mover std_msgs/Float64 -- 1000
```
Obs.: note que a unidade de medida está em milimetros.

Andar um metro para trás
```
rostopic pub -1 mover std_msgs/Float64 -- -1000
```
Parar
```
rostopic pub -1 parar std_msgs/Empty
```
Girar 60 graus a esquerda
```
rostopic pub -1 girar std_msgs/Float64 -- 60
```
Girar 60 graus a direita
```
rostopic pub -1 girar std_msgs/Float64 -- -60
```
Setar valor PWM geral 0-255
```
rostopic pub -1 setPwmLhRh std_msgs/Float64 -- 150
```
Setar valor PWM motor direito 0-255
```
rostopic pub -1 setPwmRh std_msgs/Float64 -- 150
```
Setar valor PWM motor esquerdo 0-255
```
rostopic pub -1 setPwmLh std_msgs/Float64 -- 150
```
