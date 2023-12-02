import './App.css';
import './lib/controller.js';

function App() {
  window.joypad.on('button_press', e => console.log(e));

  return (
    <div className="App">
      <header className="App-header">
        <p>
          Edit <code>src/App.j</code> and save to reload.
        </p>
      </header>
    </div>
  );
}

export default App;
