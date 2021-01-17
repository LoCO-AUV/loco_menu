# loco_menu

## Changelog
1/11: changed mini_menu to loco_menu in menu.launch  
1/14: Started work on term_display
1/17: worked on fixing bugs/getting messages via cmd line working

## Bugs
1. Appears to only read in the first menu item. 
2. menu is not sending any messages, even dummy ones, to term_display


## Notes
### Display Publisher
        Uses standard string messages of the form where each menu line is delimited by a following newline character.
### Sending Messages via Command Line
catkin_make, catkin_make install, source devel/setup.bash  
rostopic pub /loco/tags ar_recog/Tags "Message definition"  

The message definition will tab complete if you've got the messages built on your machine, and the only field you will need to update is the id field, which can be seen in this message definition ( https://github.umn.edu/IRVLab/minnebot/blob/irv_aqua/catkin_base_ws/src/aqua_related/ar_recog/msg/Tag.msg). Update that to the number you want, and you can hit enter to publish. The Tags message type, btw (https://github.umn.edu/IRVLab/minnebot/blob/irv_aqua/catkin_base_ws/src/aqua_related/ar_recog/msg/Tags.msg), is just a header and a few other things attached to an array of Tag messages. 