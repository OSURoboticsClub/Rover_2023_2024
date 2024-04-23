import ReactSlider from "react-slider";
function Slider({setThrottle}){
    return(
        <ReactSlider
        className="horizontal-slider"
        thumbClassName="example-thumb"
        trackClassName="example-track"
        
        onAfterChange={(value, index) =>
            setThrottle(value/100)
        }
        renderThumb={(props, state) => <d1 {...props}>{state.valueNow}%</d1>}
        />
    );
}
export default Slider;