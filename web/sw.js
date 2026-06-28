var CACHE_NAME = 'pipe-crawler-v6';
var SHELL_URLS = [
  '/css/theme.css',
  '/js/theme.js',
  '/images/icon-192.png',
  '/images/icon-512.png',
  '/images/apple-touch-icon.png',
  '/images/prime_inspections_logo.png',
  '/manifest.json'
];

self.addEventListener('install', function(event) {
  event.waitUntil(
    caches.open(CACHE_NAME).then(function(cache) {
      return cache.addAll(SHELL_URLS);
    })
  );
  self.skipWaiting();
});

self.addEventListener('activate', function(event) {
  event.waitUntil(
    caches.keys().then(function(names) {
      return Promise.all(
        names.filter(function(name) { return name !== CACHE_NAME; })
             .map(function(name) { return caches.delete(name); })
      );
    })
  );
  self.clients.claim();
});

self.addEventListener('fetch', function(event) {
  var url = new URL(event.request.url);

  // Never cache websocket, API, or external requests
  if (url.protocol === 'ws:' || url.protocol === 'wss:' || url.pathname.startsWith('/api/') || url.origin !== self.location.origin) {
    return;
  }

  // App HTML / navigations: ALWAYS network, NEVER cache. The page is no-store from Flask;
  // serving a cached HTML (e.g. after a reload during a network blip) would run stale app
  // JS. Network-only here guarantees the latest code, so fixes take effect on reload.
  var accept = event.request.headers.get('accept') || '';
  if (event.request.mode === 'navigate' || url.pathname === '/' || url.pathname.endsWith('.html') || accept.indexOf('text/html') !== -1) {
    event.respondWith(fetch(event.request));
    return;
  }

  // Static assets (css/js/images): network-first, fall back to cache for offline.
  event.respondWith(
    fetch(event.request).then(function(response) {
      if (event.request.method === 'GET' && response.status === 200) {
        var clone = response.clone();
        caches.open(CACHE_NAME).then(function(cache) {
          cache.put(event.request, clone);
        });
      }
      return response;
    }).catch(function() {
      return caches.match(event.request);
    })
  );
});
