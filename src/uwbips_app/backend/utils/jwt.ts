import jwt from 'jsonwebtoken';
import dotenv from 'dotenv';
dotenv.config();

export namespace JWT {
    const JWT_ACCESS_KEY: string = process.env.JWT_ACCESS_KEY as string;
    const JWT_ACCESS_EXPIRES: string = process.env.JWT_ACCESS_EXPIRES as string;
    const JWT_REFRESH_KEY: string = process.env.JWT_REFRESH_KEY as string;
    const JWT_REFRESH_EXPIRES: string = process.env.JWT_REFRESH_EXPIRES as string;

    export interface jwtVerificationType {
        is_granted: boolean;
        user: any;
        error: any;
    }

    export function generateAccessToken(user: object): string {
        return jwt.sign(user, JWT_ACCESS_KEY, { expiresIn: JWT_ACCESS_EXPIRES });
    }

    export function verifyAccessToken(token: string): jwtVerificationType {
        try {
            const decoded = jwt.verify(token, JWT_ACCESS_KEY);
            return { is_granted: true, user: decoded, error: {} };
        } catch (err) {
            return { is_granted: false, user: {}, error: err };
        }
    }

    export function generateRefreshToken(user: object): string {
        return jwt.sign(user, JWT_REFRESH_KEY, { expiresIn: JWT_REFRESH_EXPIRES });
    }

    export function verifyRefreshToken(token: string): jwtVerificationType {
        try {
            const decoded = jwt.verify(token, JWT_REFRESH_KEY);
            return { is_granted: true, user: decoded, error: {} };
        } catch (err) {
            return { is_granted: false, user: {}, error: err };
        }
    }
}
