// Joypad events handler

import emitter from './emitter';
import joypad from './joypad';
import loop from './loop';
import { EVENTS, STICKS, DIRECTIONS, BUTTON_MAPPING } from './constants';
import { findButtonMapping } from './helpers';

const initEventListeners = () => {
    window.addEventListener(EVENTS.CONNECT.NATIVE, e => {
        emitter.publish(EVENTS.CONNECT.ALIAS, e);

        // Start loop on gamepad connection if not already started
        if (!joypad.loopStarted) {
            joypad.loopStarted = true;
            return loop.start();
        }
    });
    window.addEventListener(EVENTS.DISCONNECT.NATIVE, e => {
        emitter.publish(EVENTS.DISCONNECT.ALIAS, e);

        // Remove instance and reset events on gamepad disconnection
        joypad.remove(e.gamepad.index);
        joypad.buttonEvents.joypad[e.gamepad.index] = null;

        // Stop loop if all gamepads have been disconnected
        if (!Object.keys(joypad.instances).length) {
            joypad.loopStarted = false;
            return loop.stop(loop.id);
        }
    });
    window.addEventListener(EVENTS.BUTTON_PRESS.ALIAS, e => {
        return emitter.publish(EVENTS.BUTTON_PRESS.ALIAS, e);
    });
    window.addEventListener(EVENTS.BUTTON_RELEASE.ALIAS, e => {
        return emitter.publish(EVENTS.BUTTON_RELEASE.ALIAS, e);
    });
    window.addEventListener(EVENTS.AXIS_MOVEMENT.ALIAS, e => {
        return emitter.publish(EVENTS.AXIS_MOVEMENT.ALIAS, e);
    });
};
const listenToButtonEvents = gamepad => {
    //console.log(gamepad.buttons);
    gamepad.buttons.forEach((button, index) => {
        const cacheEvents = joypad.cacheEvents
        const { customButtonMapping } = joypad.settings;
        const buttonMapping = customButtonMapping ? customButtonMapping : BUTTON_MAPPING;
        const keys = findButtonMapping(index, buttonMapping);
        const { buttonEvents } = joypad;
        if (keys && keys.length) {
            keys.forEach(key => {
                
                // If button is pressed then set press status of button
                
                if (button.pressed || button.hold) {
                    
                    //console.log(button.hold)
                    if (cacheEvents[buttonMapping[key]] === 0) {
                        buttonEvents.joypad[gamepad.index][key] = {
                            pressed: true,
                            hold: false,
                            released: false
                        };
                    }
                    if(cacheEvents[buttonMapping[key]] === 15) {
                        console.log("In here")
                        buttonEvents.joypad[gamepad.index][key] = {
                            pressed: false,
                            hold: true,
                            released: false
                        };                       
                    }
                    
                    // Set button event data
                    buttonEvents.joypad[gamepad.index][key].button = button;
                    buttonEvents.joypad[gamepad.index][key].index = index;
                    buttonEvents.joypad[gamepad.index][key].gamepad = gamepad;
                    cacheEvents[buttonMapping[key]] +=1
                    
                }

                // If button is not pressed then set release status of button
                else if (!button.pressed && buttonEvents.joypad[gamepad.index][key]) {
                    buttonEvents.joypad[gamepad.index][key].released = true;
                    buttonEvents.joypad[gamepad.index][key].hold = false;
                    cacheEvents[buttonMapping[key]] = 0
                }
            });
        }
    });
};
const listenToAxisMovements = gamepad => {
    const axisMovementEvent = eventData => new CustomEvent(EVENTS.AXIS_MOVEMENT.ALIAS, { detail: eventData });
    const { axisMovementThreshold } = joypad.settings;
    const { axes } = gamepad;
    const totalAxisIndexes = axes.length;
    const totalSticks = totalAxisIndexes / 2;

    axes.forEach((axis, index) => {
        if (Math.abs(axis) > axisMovementThreshold) {
            let stickMoved = null;
            let directionOfMovement = null;
            let axisMovementValue = axis;

            if (index < totalSticks) {
                stickMoved = STICKS.LEFT.NAME;
            } else {
                stickMoved = STICKS.RIGHT.NAME;
            }

            if (index === STICKS.LEFT.AXES.X || index === STICKS.RIGHT.AXES.X) {
                directionOfMovement = axis < 0 ? DIRECTIONS.LEFT : DIRECTIONS.RIGHT;
            }
            if (index === STICKS.LEFT.AXES.Y || index === STICKS.RIGHT.AXES.Y) {
                directionOfMovement = axis < 0 ? DIRECTIONS.TOP : DIRECTIONS.BOTTOM;
            }

            const eventData = { gamepad, totalSticks, stickMoved, directionOfMovement, axisMovementValue, axis: index };
            return window.dispatchEvent(axisMovementEvent(eventData));
        }
    });
};
const dispatchCustomEvent = (eventName, buttonEvents, buttonName) => {
    const joypadEvent = eventData => new CustomEvent(eventName, { detail: eventData });
    const { index, gamepad } = buttonEvents[buttonName];
    const eventData = {
        buttonName,
        button: buttonEvents[buttonName].button,
        index,
        gamepad
    };
    
    window.dispatchEvent(joypadEvent(eventData));
};
const handleButtonEvent = (buttonName, buttonEvents) => {
    // Fire button press event
    if (buttonEvents[buttonName].pressed) {
        dispatchCustomEvent(EVENTS.BUTTON_PRESS.ALIAS, buttonEvents, buttonName);

        // Reset button usage flags
        buttonEvents[buttonName].pressed = false;
        // Set last button press to fire button release event
        buttonEvents[buttonName].last_event = EVENTS.BUTTON_PRESS.ALIAS;
    }

    // Button being held
    else if (buttonEvents[buttonName].hold) {
        dispatchCustomEvent(EVENTS.BUTTON_HELD.ALIAS, buttonEvents, buttonName);

        buttonEvents[buttonName.pressed] = false;

        buttonEvents[buttonName].last_event = EVENTS.BUTTON_HELD.ALIAS;
    }

    // Button being released
    else if (buttonEvents[buttonName].released && buttonEvents[buttonName].last_event === EVENTS.BUTTON_PRESS.ALIAS) {
        dispatchCustomEvent(EVENTS.BUTTON_RELEASE.ALIAS, buttonEvents, buttonName);

        delete buttonEvents[buttonName];
    }
};

initEventListeners();
export { listenToButtonEvents, listenToAxisMovements, handleButtonEvent }