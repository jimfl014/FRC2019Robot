
package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class HHJoystickButtons {

    Joystick joystick;
    int numberOfButtons;
    HHJoystickButton[] joystickButtons;
    
    public HHJoystickButtons( Joystick joystick, int numberOfButtons ) {

        this.joystick = joystick;
        this.numberOfButtons = numberOfButtons;
        this.joystickButtons = new HHJoystickButton[ numberOfButtons ];

        for( int i = 0; i < numberOfButtons; i += 1 ) {

            this.joystickButtons[ i ] = new HHJoystickButton( joystick, i + 1 );
        }
    }

    public void updateState() {
        for( int i = 0; i < numberOfButtons; i += 1 ) {

            this.joystickButtons[ i ].updateState();
        }
    }
    
    public HHJoystickButtonState getState( int button ) {
        if( !( 1 <= button && button <= numberOfButtons ) ) {
            throw new IllegalArgumentException("Illegal button number: " + button);
        }

        return joystickButtons[ button - 1 ].getState();
    }

    public boolean isPressed( int button ) {
        if( !( 1 <= button && button <= numberOfButtons ) ) {
            throw new IllegalArgumentException("Illegal button number: " + button);
        }

        return joystickButtons[ button - 1 ].isPressed();
    }

    public boolean isReleased( int button ) {
        if( !( 1 <= button && button <= numberOfButtons ) ) {
            throw new IllegalArgumentException("Illegal button number: " + button);
        }

        return joystickButtons[ button - 1 ].isReleased();
    }
}
