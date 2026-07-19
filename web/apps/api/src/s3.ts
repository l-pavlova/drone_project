import { PutObjectCommand, S3Client } from "@aws-sdk/client-s3";
import { config } from "./config.js";

/** S3/MinIO client. forcePathStyle for MinIO compatibility. */
export const s3 = new S3Client({
  endpoint: config.s3.endpoint,
  region: config.s3.region,
  forcePathStyle: true,
  credentials: {
    accessKeyId: config.s3.accessKey,
    secretAccessKey: config.s3.secretKey,
  },
});

/** Store a frame image; returns the s3:// uri the worker will fetch. */
export async function putFrame(
  key: string,
  body: Buffer,
  contentType = "image/png",
): Promise<string> {
  await s3.send(
    new PutObjectCommand({
      Bucket: config.s3.bucket,
      Key: key,
      Body: body,
      ContentType: contentType,
    }),
  );
  return `s3://${config.s3.bucket}/${key}`;
}
