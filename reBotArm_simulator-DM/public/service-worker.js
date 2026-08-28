const CACHE_NAME = 'rebot-arm-pwa-v80-physics-active-snapfix';
const APP_SHELL = [
  '/',
  '/index.html',
  '/manifest.webmanifest',
  '/favicon.png',
  '/css/rebot-sim.css?v=20260807-i18n1',
  '/js/pwa.js?v=20260612-fakecarry1',
  '/js/i18n.js?v=20260828-physics-active-3',
  '/js/rebot-sim.js?v=20260828-physics-active-3',
  '/js/ros/rebot-ros-client.js?v=20260807-i18n1',
  '/js/ros/rebot-ros-ui.js?v=20260828-physics-active-3',
  '/js/rebot-llm.js?v=20260807-i18n1',
  '/lib/three-r128.min.js',
  '/lib/STLLoader-umd.js',
  '/lib/URDFLoader.js'
];

self.addEventListener('install', (event) => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then((cache) => cache.addAll(APP_SHELL))
      .then(() => self.skipWaiting())
  );
});

self.addEventListener('activate', (event) => {
  event.waitUntil(
    caches.keys()
      .then((keys) => Promise.all(
        keys.filter((key) => key !== CACHE_NAME).map((key) => caches.delete(key))
      ))
      .then(() => self.clients.claim())
  );
});

self.addEventListener('fetch', (event) => {
  const request = event.request;
  const url = new URL(request.url);

  if (request.method !== 'GET' || url.origin !== self.location.origin) return;

  if (
    url.pathname.startsWith('/api/')
    || request.mode === 'navigate'
    || url.pathname.endsWith('.html')
    || url.pathname.endsWith('.js')
    || url.pathname.endsWith('.css')
  ) {
    event.respondWith(fetch(request));
    return;
  }

  event.respondWith(
    caches.match(request).then((cached) => {
      if (cached) return cached;
      return fetch(request).then((response) => {
        if (!response || response.status !== 200) return response;
        const copy = response.clone();
        caches.open(CACHE_NAME).then((cache) => cache.put(request, copy));
        return response;
      });
    })
  );
});
