import ReactSlider from "react-slider";
function Slider(props){
    return(
        <ReactSlider
        className="horizontal-slider"
        thumbClassName="example-thumb"
        trackClassName="example-track"
        
        onAfterChange={(value, index) =>
            props.setThrottle(value/100)
        }
        renderThumb={(props, state) => <div {...props}>{state.valueNow}</div>}
        />
    );
}
export default Slider;