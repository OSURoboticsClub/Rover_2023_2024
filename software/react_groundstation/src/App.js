import './App.css';
import './lib/controller.js';
import MainPage from './pages/MainPage.js';



function App() {
  

  window.joypad.on('button_press', e => console.log(e));
  window.joypad.on('button_held', e => console.log(e));
  
  

  return (
    <div className="App">
      <MainPage/>
    </div>
  );
}

export default App;
