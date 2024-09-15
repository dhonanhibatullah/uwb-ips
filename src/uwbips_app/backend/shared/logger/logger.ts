import fs from 'fs';
import path from 'path';
import { createLogger, format, transports } from 'winston';

const { combine, timestamp, printf, colorize } = format;

const current_date: Date = new Date();
const foldername: string =
    current_date.getFullYear().toString() + '-' + current_date.getMonth().toString() + '-' + current_date.getDate().toString() + '_logs';
const filename: string =
    current_date.getHours().toString() + ':' + current_date.getMinutes().toString() + ':' + current_date.getSeconds().toString() + '.log';

let log_dir: string = path.join(__dirname, 'log');
if (!fs.existsSync(log_dir)) fs.mkdirSync(log_dir);
log_dir = path.join(log_dir, foldername);
if (!fs.existsSync(log_dir)) fs.mkdirSync(log_dir);

const logFormat = printf(({ level, message, timestamp }) => {
    return `${timestamp} [${level.toUpperCase()}]: ${message}`;
});

const logger = createLogger({
    format: combine(timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }), logFormat),
    transports: [
        new transports.Console({
            format: logFormat,
        }),
        new transports.File({
            filename: path.join(log_dir, filename),
            level: 'info',
        }),
    ],
});

export default logger;
