# AMG8833-Occupancy-Detection
This uses an AMG8833 thermal array sensor and ESP32 mounted at a doorway to detect people coming and leaving out a room. It then tallys the people to keep track on how many people are present in the room to detect occupancy

I started out with just the thermal array and got very good results. It was consinstently keeping track on how many people were in the room. Where it struggled though was when I was wearing a jacket, as I released less heat, it wasn't able to track me as well.

Thats when I had the idea to attach a ToF sensor (VL53L8CX) as well to increase the sensitivity of the thermal sensor when there is an something there. This did help but didn't eliminate the issue. 

Currently I use a mmwave sensor to detect presense, however mmwave does have its drawbacks. Namely being fans and other moving inanimate objects get detected as well and when I am sleeping it tends to lose track. I still belive some sensor mounted at a doorway is the best as it eliminates these flaws. 

There is a [new sensor](https://www.st.com/en/imaging-and-photonics-solutions/vl53l9cx.html#sample-buy) being released by STM electronics in the new future which is essentially a LiDAR sensor and has many more data points to work with. I am hoping that can help eliminate the issue I was having with the thermal sensor. 
