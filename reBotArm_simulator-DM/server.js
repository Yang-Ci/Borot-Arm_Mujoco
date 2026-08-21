const http = require('http');
const https = require('https');
const fs = require('fs');
const os = require('os');
const path = require('path');

const { URL } = require('url');

// Load .env file (does not override existing env vars)
(function loadEnv() {
  const envPath = path.join(__dirname, '.env');
  try {
    if (fs.existsSync(envPath)) {
      const lines = fs.readFileSync(envPath, 'utf8').split('\n');
      for (const line of lines) {
        const trimmed = line.trim();
        if (!trimmed || trimmed.startsWith('#')) continue;
        const eq = trimmed.indexOf('=');
        if (eq < 1) continue;
        const key = trimmed.slice(0, eq).trim();
        const val = trimmed.slice(eq + 1).trim().replace(/^["']|["']$/g, '');
        if (!(key in process.env)) process.env[key] = val;
      }
    }
  } catch (_) {}
})();

const USE_HTTPS = process.env.HTTPS === '1';
const PORT = Number(process.env.PORT || (USE_HTTPS ? 3443 : 3001));
const ROOT = __dirname;
const PUBLIC_DIR = path.join(ROOT, 'public');
const BRINGUP_DIR = path.resolve(
  path.join(ROOT, '..', 'reBotArmController_ROS2-main', 'src', 'rebotarm_bringup')
);
const URDF_FILE = path.join(BRINGUP_DIR, 'description', 'urdf', 'reBot-DevArm_colored.urdf');
const MESHES_DIR = path.join(BRINGUP_DIR, 'description', 'meshes');
const GRIPPER_MESHES_DIR = path.join(ROOT, 'split_meshes', 'grouped_gripper');
const DEFAULT_KEY_FILE = path.join(ROOT, '.certs', 'rebotarm-local-server.key');
const DEFAULT_CERT_FILE = path.join(ROOT, '.certs', 'rebotarm-local-server.crt');

// MCP/LLM 配置（前端只负责代理到虚拟机的 text-agent HTTP 服务）
const DEFAULT_TEXT_AGENT_URL = process.env.REBOTARM_TEXT_AGENT_URL || 'http://localhost:8082';
const DEFAULT_MCP_URL = process.env.REBOTARM_MCP_URL || 'http://localhost:8081/mcp';

const MIME_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.js': 'application/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.webmanifest': 'application/manifest+json; charset=utf-8',
  '.png': 'image/png',
  '.stl': 'model/stl',
  '.STL': 'model/stl',
  '.urdf': 'application/xml; charset=utf-8',
  '.xml': 'application/xml; charset=utf-8'
};

function send(res, status, body, type) {
  res.writeHead(status, {
    'Content-Type': type || 'text/plain; charset=utf-8',
    'Cache-Control': 'no-store'
  });
  res.end(body);
}

function sendJson(res, status, body) {
  send(res, status, JSON.stringify(body, null, 2), MIME_TYPES['.json']);
}

function sendFile(res, filePath) {
  fs.stat(filePath, (statErr, stat) => {
    if (statErr || !stat.isFile()) {
      sendJson(res, 404, { error: 'File not found' });
      return;
    }

    const ext = path.extname(filePath);
    res.writeHead(200, {
      'Content-Type': MIME_TYPES[ext] || 'application/octet-stream',
      'Content-Length': stat.size,
      'Cache-Control': ext.toLowerCase() === '.stl' ? 'public, max-age=3600' : 'no-store'
    });
    fs.createReadStream(filePath).pipe(res);
  });
}

function safePublicPath(urlPath) {
  const cleanPath = decodeURIComponent(urlPath.split('?')[0]);
  const relative = cleanPath === '/' ? 'index.html' : cleanPath.replace(/^\/+/, '');
  const filePath = path.resolve(path.join(PUBLIC_DIR, relative));
  if (!filePath.startsWith(PUBLIC_DIR)) return null;
  return filePath;
}

function sendMesh(res, filename) {
  const safeName = path.basename(filename);
  sendFile(res, path.join(MESHES_DIR, safeName));
}

function sendGripperMesh(res, filename) {
  const safeName = path.basename(filename);
  sendFile(res, path.join(GRIPPER_MESHES_DIR, safeName));
}

function getLanAddresses() {
  return Object.values(os.networkInterfaces())
    .flat()
    .filter((item) => item && item.family === 'IPv4' && !item.internal)
    .map((item) => item.address);
}

function requestHandler(req, res) {
  const urlPath = req.url.split('?')[0];

  // MCP 配置端点
  if (urlPath === '/api/mcp/config') {
    sendJson(res, 200, {
      textAgentUrl: DEFAULT_TEXT_AGENT_URL,
      mcpUrl: DEFAULT_MCP_URL
    });
    return;
  }

  // LLM 聊天代理端点 → 转发到虚拟机的 text-agent HTTP 服务
  if (urlPath === '/api/llm/chat' && req.method === 'POST') {
    handleTextAgentChat(req, res);
    return;
  }

  // Text-agent 健康检查
  if (urlPath === '/api/llm/health' && req.method === 'GET') {
    handleTextAgentHealth(req, res);
    return;
  }

  if (urlPath === '/api/config') {
    sendJson(res, 200, {
      name: 'reBot Arm B601-DM',
      frame: {
        rosX: 'forward',
        rosY: 'left',
        rosZ: 'up',
        threeMapping: { x: 'ros_x', y: 'ros_z', z: '-ros_y' }
      },
      reachMeters: 0.65,
      payloadKg: 1.5,
      gripper: {
        name: 'gripper',
        motorId: '0x07',
        closedMeters: 0,
        openMeters: 0.09,
        visualOpenMeters: 0.057,
        rosService: '/rebotarm/gripper/set'
      }
    });
    return;
  }

  if (urlPath === '/api/urdf') {
    sendFile(res, URDF_FILE);
    return;
  }

  const meshMatch = urlPath.match(/^\/api\/(?:description\/)?meshes\/(.+)$/);
  if (meshMatch) {
    sendMesh(res, meshMatch[1]);
    return;
  }

  const gripperMeshMatch = urlPath.match(/^\/api\/gripper_meshes\/(.+)$/);
  if (gripperMeshMatch) {
    sendGripperMesh(res, gripperMeshMatch[1]);
    return;
  }

  const filePath = safePublicPath(urlPath);
  if (!filePath) {
    sendJson(res, 403, { error: 'Forbidden' });
    return;
  }

  sendFile(res, filePath);
}

function createServer() {
  if (!USE_HTTPS) return http.createServer(requestHandler);

  const keyFile = process.env.HTTPS_KEY || DEFAULT_KEY_FILE;
  const certFile = process.env.HTTPS_CERT || DEFAULT_CERT_FILE;

  if (!fs.existsSync(keyFile) || !fs.existsSync(certFile)) {
    console.error(`HTTPS certificate not found: ${keyFile} / ${certFile}`);
    console.error('Run: npm run cert:dev');
    process.exit(1);
  }

  return https.createServer({
    key: fs.readFileSync(keyFile),
    cert: fs.readFileSync(certFile)
  }, requestHandler);
}

const server = createServer();

// 读取请求体
function readBody(req) {
  return new Promise((resolve, reject) => {
    let body = '';
    req.on('data', chunk => { body += chunk; });
    req.on('end', () => resolve(body));
    req.on('error', reject);
  });
}

// HTTP 请求代理
function proxyRequest(targetUrl, options, body, timeoutMs = 15000) {
  return new Promise((resolve, reject) => {
    let url;
    try {
      url = new URL(targetUrl);
    } catch (e) {
      reject(new Error(`Invalid URL: ${targetUrl}`));
      return;
    }
    const lib = url.protocol === 'https:' ? https : http;
    const reqOptions = {
      hostname: url.hostname,
      port: url.port || (url.protocol === 'https:' ? 443 : 80),
      path: url.pathname + url.search,
      method: options.method || 'POST',
      timeout: timeoutMs,
      headers: {
        'Content-Type': 'application/json',
        'Content-Length': Buffer.byteLength(body),
        ...options.headers
      }
    };

    const proxyReq = lib.request(reqOptions, (proxyRes) => {
      let data = '';
      // SSE 是流式响应，收到 headers 后立即返回初始数据，不等 end
      const contentType = proxyRes.headers['content-type'] || '';
      const isSSE = contentType.includes('text/event-stream');

      if (isSSE) {
        // 流式响应：读取直到遇到第一个完整事件或超时
        let buffer = '';
        const readTimeout = setTimeout(() => {
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: buffer
          });
          proxyReq.destroy();
        }, timeoutMs);

        proxyRes.on('data', chunk => {
          buffer += chunk.toString('utf8');
          // 如果收到完整的事件（data: ... \n\n），立即返回
          if (buffer.includes('\n\n')) {
            clearTimeout(readTimeout);
            resolve({
              status: proxyRes.statusCode,
              headers: proxyRes.headers,
              body: buffer
            });
            proxyReq.destroy();
          }
        });
        proxyRes.on('end', () => {
          clearTimeout(readTimeout);
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: buffer
          });
        });
        proxyRes.on('error', (err) => {
          clearTimeout(readTimeout);
          reject(err);
        });
      } else {
        proxyRes.on('data', chunk => { data += chunk; });
        proxyRes.on('end', () => {
          resolve({
            status: proxyRes.statusCode,
            headers: proxyRes.headers,
            body: data
          });
        });
        proxyRes.on('error', (err) => reject(err));
      }
    });

    proxyReq.on('timeout', () => {
      proxyReq.destroy();
      reject(new Error(`Request timeout after ${timeoutMs}ms: ${targetUrl}`));
    });
    proxyReq.on('error', reject);

    if (body) {
      proxyReq.write(body);
    }
    proxyReq.end();
  });
}

// 代理到虚拟机的 text-agent HTTP 服务
async function handleTextAgentChat(req, res) {
  try {
    const body = await readBody(req);
    const { text, message, reset } = JSON.parse(body);
    const userText = text || message || '';
    if (!userText) {
      sendJson(res, 400, { ok: false, error: 'empty text' });
      return;
    }

    const targetUrl = `${DEFAULT_TEXT_AGENT_URL.replace(/\/+$/, '')}/chat`;
    const payload = JSON.stringify({ text: userText, reset: !!reset });

    console.log(`[text-agent] POST ${targetUrl} text="${userText.substring(0, 40)}"`);

    const response = await proxyRequest(targetUrl, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    }, payload, 90000);

    console.log(`[text-agent] response status=${response.status} body_len=${response.body.length}`);

    res.writeHead(response.status, { 'Content-Type': 'application/json; charset=utf-8' });
    res.end(response.body);
  } catch (error) {
    console.error('[text-agent] error:', error.message);
    sendJson(res, 502, { ok: false, error: `无法连接到 text-agent: ${error.message}` });
  }
}

async function handleTextAgentHealth(req, res) {
  try {
    const targetUrl = `${DEFAULT_TEXT_AGENT_URL.replace(/\/+$/, '')}/health`;
    const response = await proxyRequest(targetUrl, {
      method: 'GET',
      headers: {}
    }, '', 5000);
    res.writeHead(response.status, { 'Content-Type': 'application/json' });
    res.end(response.body);
  } catch (error) {
    sendJson(res, 502, { ok: false, error: `text-agent 不可达: ${error.message}` });
  }
}

server.listen(PORT, () => {
  const protocol = USE_HTTPS ? 'https' : 'http';
  const lanAddresses = getLanAddresses();
  console.log('========================================');
  console.log('  reBot Arm B601-DM Simulator Started');
  console.log('========================================');
  console.log(`  Local: ${protocol}://localhost:${PORT}`);
  lanAddresses.forEach((address) => console.log(`  LAN:   ${protocol}://${address}:${PORT}`));
  console.log(`  URDF:  ${protocol}://localhost:${PORT}/api/urdf`);
  console.log(`  Mesh:  ${protocol}://localhost:${PORT}/api/description/meshes/base_link.STL`);
  console.log(`  Gripper meshes: ${GRIPPER_MESHES_DIR}`);
  console.log('----------------------------------------');
  console.log(`  URDF file: ${URDF_FILE}`);
  console.log(`  Mesh dir:  ${MESHES_DIR}`);
});
