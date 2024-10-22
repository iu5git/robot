import {BrowserRouter, Routes, Route, Link, useParams} from 'react-router-dom';
import {FunctionComponent} from 'react';
import {pages} from './pages';

const Router: FunctionComponent = () => (
  <BrowserRouter>
    <Routes>
      {Object.entries(pages).map(([name, {link, component}], i) => (
        <Route key={i} path={link} element={component({})}/>
      ))}
    </Routes>
  </BrowserRouter>
);

function Topics() {
  // let match = useRouteMatch();

  return (
    <div>
      <h2>Topics</h2>

      <ul>
        <li>
          <Link to={`components`}>
            Components
          </Link>
        </li>
        <li>
          <Link to={`props-v-state`}>
            Props v. State
          </Link>
        </li>
      </ul>

      <Routes>
        <Route path={`:topicId`} element={<Topic />}/>
        <Route path={'*'} element={
          <h3>Please select a topic.</h3>
        }/>
      </Routes>
    </div>
  );
}

function Topic() {
  let { topicId } = useParams();
  return <h3>Requested topic ID: {topicId}</h3>;
}

export default Router;
