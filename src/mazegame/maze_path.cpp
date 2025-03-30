#include "mazegame/maze_path.h"
#include "competition/autonomous.h"
#include "robot-config.h"

void maze_path() {
    CommandController cc {
        // Path here

        /* Pseudo Code
        (x, y, rot)
        
        Start       (108, 12, 0)
        Forward 24  (132, 12, 0)
        Right 90    (132, 12, 90)
        Forward 72  (132, 84, 90)
        Right 90    (132, 84, 180)
        Forward 48  (84, 84, 180)
        Right 90    (84, 84, 270)
        Forward 24  (84, 60, 270)
        Right 90    (84, 60, 0)
        Forward 24  (108, 60, 0)
        Left 90     (108, 60, 270)
        Forward 24  (108, 36, 270)
        Left 90     (108, 36, 180)
        Forward 72  (36, 36, 180)
        Left 90     (36, 36, 270)
        Forward 24  (36, 60, 270)
        Left 90     (36, 60, 0)
        Forward 24  (60, 60, 0)
        Right 90    (60, 60, 90)
        Forward 24  (60, 84, 90)
        Right 90    (60, 84, 180)
        Forward 48  (12, 84, 180)
        Right 90    (12, 84, 270)
        Forward 72  (12, 12, 270)
        Right 90    (12, 12, 0)
        Forward 60  (72, 12, 0)
        
        */
    };
    
    cc.run();
}