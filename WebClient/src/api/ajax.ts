import {ajaxDebug} from '../base/globals';

// ip: 89.208.197.247
// domain: sabernews

// Поменять тут, в server/server.js, server/server-api.js (не забыть CORS)
// const APIurl = '127.0.0.1';
// Локальная разработка
const APIurl = 'http://localhost:8081/api/';

// CSRF guard
// const CSRFCookieName = '_csrf';

type AjaxMethodT = 'POST' | 'GET';
type AjaxMethodsT = {
  [key: string]: AjaxMethodT,
}

const ajaxMethods: AjaxMethodsT = {
  post: 'POST',
  get: 'GET',
};

/**
 * Поддерживаемые http-статусы ответа: 200, 400, 404
 */
const ajaxStatuses = {
  ok: 200,
  notFound: 404,
  redirect: 303,
  badRequest: 400,
  invalidSession: 424,
};

interface RequestParamsNoMethodI {
  url: string,
  body?: object,
}

interface RequestParamsI extends RequestParamsNoMethodI {
  method: AjaxMethodT,
}

interface AjaxResponse {
  status: number,
  response: {
    data: any,
    msg?: string,
    status: number,
  },
}

/**
 * Выполняет ajax-запрос на сервер. При успешном выполнении вызывает callback
 * @param {RequestParamsI} requestParams
 * @property {AjaxMethod} [method = "GET"]
 * @property {Url} [url = '/']
 * @property {any} body
 * @return {Promise}
 */
function ajax(requestParams: RequestParamsI): Promise<AjaxResponse> {
  const url = APIurl + (requestParams.url || '/');

  // CSRF guard
  // const csrf = document.cookie.split(';')
  //     .map((c) => c.trim())
  //     .find((c) => c.startsWith(CSRFCookieName + '='))
  //     ?.substring(CSRFCookieName.length + 1);  // skip '_csrf='

  const fetchParams: RequestInit = {
    body: JSON.stringify(requestParams.body),
    mode: 'cors',
    credentials: 'include',
    headers: {
      'Content-Type': 'application/json',
      // 'X-XSRF-TOKEN': csrf,
    },
    method: requestParams.method,
  };

  if (ajaxDebug) {
    console.log('ajax request', {url}, ': ' + JSON.stringify(fetchParams));
  }

  let status = 0;
  return fetch(url, fetchParams)
      .then((response: Response) => {
        status = response.status;
        return response.json();
      })
      .then((response: object) => {
        if (ajaxDebug) {
          console.log('ajax resolved ' + status + ': ');
          console.log(response);
        }
        return {
          status,
          response,
        };
      })
      .catch((error: Error) => {
        console.warn(error);
      }) as Promise<AjaxResponse>;
}

/**
 * Выполняет отправку фото на сервер. При успешном выполнении вызывает callback
 * @param {Object} requestParams
 * @property {Url} [url = '/']
 * @property {any} body
 * @return {Promise}
 */
function postFile(requestParams: RequestParamsNoMethodI): Promise<AjaxResponse> {
  const url = APIurl + (requestParams.url || '/');
  const formData = new FormData();
  formData.append('img', requestParams.body as Blob);
  const fetchParams: RequestInit = {
    body: formData,
    mode: 'cors',
    credentials: 'include',
    method: ajaxMethods.post,
  };

  if (ajaxDebug) {
    console.log('ajax file post request', {url},
        ': ' + JSON.stringify(fetchParams));
  }

  let status = 0;
  return fetch(url, fetchParams)
      .then((response) => {
        status = response.status;
        return response.json();
      })
      .then((response) => {
        if (ajaxDebug) {
          console.log('ajax resolved ' + status + ': ');
          console.log(response);
        }
        return {
          status,
          response,
        };
      })
      .catch((error: Error) => {
        console.warn(error);
      }) as Promise<AjaxResponse>;
}

// Плагин для общения с API
const Ajax = {
  AJAX_METHODS: ajaxMethods,
  STATUS: ajaxStatuses,
  get: (requestParams: RequestParamsNoMethodI) => ajax({method: ajaxMethods.get, ...requestParams}),
  post: (requestParams: RequestParamsNoMethodI) => ajax({method: ajaxMethods.post, ...requestParams}),
  postFile,
  APIurl,
};

export default Ajax;
