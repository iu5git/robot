import { FC, useState } from "react";
import './spinner.scss';

export type SpinnerClassesT = 'offline' | 'online' | 'loading' | 'error';

interface spinnerProps {
  status: SpinnerClassesT;
}

const SpinnerComponent: FC<spinnerProps> = (props) => {
  // const [status, setStatus] = useState('offline');

  return (
    <>
      {/* <button className='btn'onClick={()=>{
        if (status === 'offline') setStatus('online')
        else setStatus('offline')
      }}>
        on/off
      </button>

      <button className='btn'onClick={()=>{
        setStatus('error')
      }}>
        error
      </button>

      <button className='btn'onClick={()=>{
        setStatus('loading')
      }}>
        load
      </button> */}

      <div className={`loader ${props.status}`}></div>
    </>
  );
};

export default SpinnerComponent;
