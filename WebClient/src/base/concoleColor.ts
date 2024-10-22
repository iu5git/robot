const coloredTags: {
  [key: string]: string,
} = {
  'WS': '#6495ed',  // cornflower (sky-blue)
};

const processPrint = (oldCons: Console, method: 'log' | 'info' | 'warn' | 'error', ...data: any[]) => {
  const possibleTag = data[0];
  if (typeof possibleTag === 'string') {
    const lbrkt = possibleTag.indexOf('[');
    const rbrkt = possibleTag.indexOf(']');
    if (lbrkt !== -1 && rbrkt !== -1 && lbrkt < rbrkt) {
      const tag = possibleTag.substring(lbrkt + 1, rbrkt);
      if (tag in coloredTags) {
        const data_0 = [
          possibleTag.substring(0, lbrkt) + '%c[' + tag + ']',
          'color: ' + coloredTags[tag] + '; font-weight: bold;',
          possibleTag.substring(rbrkt + 1),
        ];
        oldCons[method](...data_0, ...(data.slice(1)));
        return;
      }
    }
  }
  oldCons[method](...data);
}

const wrapConsoleLogWithColoredTags = (function(oldCons) {
  return {
    ...oldCons,
    log: function(...data: any[]) {
      processPrint(oldCons, 'log', ...data)
    },
    warn: function (...data: any[]) {
      processPrint(oldCons, 'warn', ...data)
    },
  };
}(console));

export default wrapConsoleLogWithColoredTags;
