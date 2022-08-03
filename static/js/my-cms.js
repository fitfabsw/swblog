import CMS from "netlify-cms-app";
// This global flag enables manual initialization.
//window.CMS_MANUAL_INIT = true
// Usage with import from npm package
// Usage with script tag
//const { CMS, initCMS: init } = window
/**
 * Initialize without passing in config - equivalent to just importing
 * Netlify CMS the old way.
 */
//init()
CMS.init()

CMS.registerEventListener({
  name: 'preSave',
  handler: ({ entry }) => {
    console.log("GGGGGGGGGGGGGGGGGGGGGGGGG");
    return entry.get('data').set('title', 'new title');
  },
});
