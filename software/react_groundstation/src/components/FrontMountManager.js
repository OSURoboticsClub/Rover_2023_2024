import React,{useState} from 'react';
import ArmControl from './ArmControl.js'
import MiningControl from './MiningControl.js'


function FrontMountManager(props){
    const [layout,setLayout] = useState('');
    let ui = <p></p>;
    if(layout === "arm"){
        ui = <ArmControl ros = {props.ros}/>
    } else if(layout === "mining"){
        ui = <MiningControl ros = {props.ros}/>
    } else {
        ui = <h1> No UI selected!</h1>
    }
    return (
        <div>
            <label htmlFor="layout">Select Layout: </label>
            <select name="layout"
                value={layout}
                onChange={e => setLayout(e.target.value)}
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
