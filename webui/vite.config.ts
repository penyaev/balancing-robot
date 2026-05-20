import { defineConfig, type ViteDevServer, type PluginOption } from "vite";
import { mkdirSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

// Dev-only middleware that lets the in-browser "Copy debug" button drop
// the debug JSON onto the developer's filesystem. The agent collaborating
// on tuning has Read access to this repo but no network reach into the
// dev server, so a file at a stable path is the bridge:
//
//   POST /__debug/dump   body: <any JSON>   -> writes debug-dumps/latest.json
//
// Single overwriting file by design (decided 2026-05-08): no history, no
// timestamping. The file is gitignored (see root .gitignore). This plugin
// is inert in production builds because it only attaches in configureServer.
const DUMP_PATH = resolve(process.cwd(), "..", "debug-dumps", "latest.json");

function debugDumpPlugin(): PluginOption {
  return {
    name: "debug-dump-bridge",
    configureServer(server: ViteDevServer) {
      server.middlewares.use("/__debug/dump", (req, res, next) => {
        if (req.method !== "POST") return next();
        const chunks: Buffer[] = [];
        req.on("data", (c: Buffer) => chunks.push(c));
        req.on("end", () => {
          try {
            const body = Buffer.concat(chunks).toString("utf8");
            // Validate before writing so we don't persist garbage that
            // would silently confuse later reads.
            JSON.parse(body);
            mkdirSync(resolve(DUMP_PATH, ".."), { recursive: true });
            writeFileSync(DUMP_PATH, body);
            res.statusCode = 204;
            res.end();
          } catch (e) {
            res.statusCode = 400;
            res.end(String(e));
          }
        });
      });
    },
  };
}

export default defineConfig({
  server: { port: 5173, open: false },
  plugins: [debugDumpPlugin()],
});
