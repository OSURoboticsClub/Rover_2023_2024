import React,{useState,useEffect} from 'react';

import ArmControl from './ArmControl.js'
import MiningControl from './MiningControl.js'


function FrontMountManager(props){
    const [layout,setLayout] = useState('');
    const [ui,setUI] = useState(<h1> No UI selected!</h1>)
    
    const [controlState,setControlState] = useState([false,false]);

    useEffect(() => { 
        if(layout==="arm"){
            setUI(<ArmControl ros = {props.ros} controlArm = {controlState[0]}/>);
        } else if (layout === "mining"){
            setUI(<MiningControl ros = {props.ros} controlMining = {controlState[1]}/>)
        } else {
            setUI(<h1> No UI selected!</h1>)
        }
    }, [controlState]);
    
    const layoutConfig = (update) => {
        setLayout(update)
        if(update === "arm"){
            setControlState([true,false])
        } else if(update === "mining"){
            setControlState([false,true])
        } else {
            setControlState([false,false])
        }
    }
    
    return (
        <div class = "frontMountManager">
            <label htmlFor="layout">Select Layout: </label>
            
            <select name="layout"
                value={layout}
                onChange={e => layoutConfig(e.target.value)}
                id="layout"
                required>
                <option value = ""> </option>
                <option value="arm">Arm</option>
                <option value="mining">Mining</option>
            </select>
            
            {ui}
        </div>
    );
}

export default FrontMountManager;
