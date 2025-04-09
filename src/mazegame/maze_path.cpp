#include "mazegame/maze_path.h"
#include "competition/autonomous.h"
#include "robot-config.h"

void maze_auto() {
    CommandController cc {
        // Path here

        /* Pseudo Code
        (x, y, rot)
        origin is bot left
        
        Start       (108, 84, 0)
        Forward 24  (132, 84, 0)
        Right 90    (132, 84, 90)
        Forward 72  (132, 12, 90)
        Right 90    (132, 12, 180)
        Forward 48  (84, 12, 180)
        Right 90    (84, 12, 270)
        Forward 24  (84, 36, 270)
        Right 90    (84, 36, 0)
        
        Forward 24  (108, 36, 0)
        Left 90     (108, 36, 270)
        Forward 24  (108, 60, 270)
        Left 90     (108, 60, 180)
        Forward 72  (36, 60, 180)
        Left 90     (36, 60, 90)
        Forward 24  (36, 36, 90)
        Left 90     (36, 36, 0)

        Forward 24  (60, 36, 0)
        Right 90    (60, 36, 90)
        Forward 24  (60, 12, 90)
        Right 90    (60, 12, 180)
        Forward 48  (12, 12, 180)
        Right 90    (12, 12, 270)
        Forward 72  (12, 84, 270)
        Right 90    (12, 84, 0)
        Forward 60  (72, 84, 0)
        
        */

        drive_sys.DriveForwardCmd(24, vex::forward, .5),
        drive_sys.TurnToHeadingCmd(90),
        drive_sys.DriveForwardCmd(72),
        drive_sys.TurnToHeadingCmd(180),
        drive_sys.DriveForwardCmd(48),
        drive_sys.TurnToHeadingCmd(270),
        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(0),

        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(270),
        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(180),
        drive_sys.DriveForwardCmd(72),
        drive_sys.TurnToHeadingCmd(90),
        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(0),

        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(90),
        drive_sys.DriveForwardCmd(24),
        drive_sys.TurnToHeadingCmd(180),
        drive_sys.DriveForwardCmd(48),
        drive_sys.TurnToHeadingCmd(270),
        drive_sys.DriveForwardCmd(72),
        drive_sys.TurnToHeadingCmd(0),
        drive_sys.DriveForwardCmd(60)
    };
    
    cc.run();
}