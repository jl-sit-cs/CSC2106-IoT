## Run the broker with 

File edit
```
# mosquitto.conf

listener 1883 0.0.0.0
allow_anonymous true
```

```
 mosquitto -v -c "C:\Program Files\mosquitto\mosquitto.conf"
 ```