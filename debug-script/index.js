const SerialPort = require("serialport");
const http = require("http");
const Koa = require("koa");
const Router = require("@koa/router");

class httpSerialPort {
  constructor() {
    this.initWeb();
  }

  getSerialList = async (ctx) => {
    const list = await SerialPort.list();
    ctx.body = list;
  };

  initWeb = () => {
    this.app = new Koa();
    const router = new Router();
    router.get("/", (ctx, next) => {
      // ctx.router available
    });
    router.get("/api/serial/list", this.getSerialList);
    this.app.use(router.routes()).use(router.allowedMethods());
    http.createServer(this.app.callback()).listen(3000);
  };
}

new httpSerialPort();
