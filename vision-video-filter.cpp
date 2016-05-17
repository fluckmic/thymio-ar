#include "vision-video-filter.h"

#include <atomic>
#include <functional>
#include <QDebug>
#include <QFile>
#include <QSettings>
#include <QThread>
#include <QOpenGLExtraFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#include <opencv2/core.hpp>
#ifdef THYMIO_AR_IMWRITE
#include <opencv2/imgcodecs.hpp>
#endif

#include "thymio-tracker/src/ThymioTracker.h"




const auto outputHeight(480);
const auto NaN(std::numeric_limits<double>::quiet_NaN());




template<typename T>
class TripleBuffer {
public:
	TripleBuffer(): buffers(), next(next_t{0, false}), write(1), read(2) {}

	T& writeBuffer() {
		return buffers[write];
	}
	bool writeSwap() {
		auto next(this->next.exchange(next_t{write, true}));
		write = next.buffer;
		return next.valid;
	}
	bool readSwap() {
		auto next(this->next.exchange(next_t{read, false}));
		read = next.buffer;
		return next.valid;
	}
	T& readBuffer() {
		return buffers[read];
	}

private:
	typedef std::array<T, 3> buffers_t;
	typedef std::uint_fast8_t index_t;
	buffers_t buffers;
	struct next_t {
		std::uint_fast8_t buffer;
		bool valid;
	};
	std::atomic<next_t> next;
	index_t write;
	index_t read;
};




class Runnable : public QObject {
	Q_OBJECT
public:
	explicit Runnable(QThread& thread, const std::function<bool()>& function);
signals:
	void invoke();
	void ran();
private slots:
	void run();
private:
	std::function<bool()> function;
};

Runnable::Runnable(QThread& thread, const std::function<bool()>& function): function(function) {
	moveToThread(&thread);
	connect(this, &Runnable::invoke, this, &Runnable::run);
}

void Runnable::run() {
	if (function()) {
		emit ran();
	}
}




static int getCvType(QVideoFrame::PixelFormat pixelFormat) {
	switch(pixelFormat) {
	case QVideoFrame::Format_ARGB32:
		return CV_8UC4;
	case QVideoFrame::Format_ARGB32_Premultiplied:
		return CV_8UC4;
	case QVideoFrame::Format_RGB32:
		return CV_8UC4;
	case QVideoFrame::Format_RGB24:
		return CV_8UC3;
	case QVideoFrame::Format_RGB565:
		return CV_8UC2;
	case QVideoFrame::Format_RGB555:
		return CV_8UC2;
	case QVideoFrame::Format_ARGB8565_Premultiplied:
		return CV_8UC3;
	case QVideoFrame::Format_BGRA32:
		return CV_8UC4;
	case QVideoFrame::Format_BGRA32_Premultiplied:
		return CV_8UC4;
	case QVideoFrame::Format_BGR32:
		return CV_8UC4;
	case QVideoFrame::Format_BGR24:
		return CV_8UC3;
	case QVideoFrame::Format_BGR565:
		return CV_8UC2;
	case QVideoFrame::Format_BGR555:
		return CV_8UC2;
	case QVideoFrame::Format_BGRA5658_Premultiplied:
		return CV_8UC3;
	case QVideoFrame::Format_YUV420P:
		return CV_8UC1;
	default:
		qCritical() << pixelFormat;
		qFatal("unknown pixel format");
	}
}

static cv::ColorConversionCodes getCvtCode(QVideoFrame::PixelFormat pixelFormat) {
	switch (pixelFormat) {
	case QVideoFrame::Format_BGR32:
		return cv::COLOR_BGR2GRAY;
	case QVideoFrame::Format_YUV420P:
		// no conversion: just take the Y
		return cv::COLOR_COLORCVT_MAX;
	default:
		qCritical() << pixelFormat;
		qFatal("unknown pixel format");
	}
}

QMatrix4x4 cvAffine3dToQMatrix4x4(bool valid, const cv::Affine3d& affine) {
	if (!valid) {
		return QMatrix4x4();
	}
	QMatrix4x4 matrix(
		affine.matrix.val[0], affine.matrix.val[1], affine.matrix.val[2], affine.matrix.val[3],
		-affine.matrix.val[4], -affine.matrix.val[5], -affine.matrix.val[6], -affine.matrix.val[7],
		-affine.matrix.val[8], -affine.matrix.val[9], -affine.matrix.val[10], -affine.matrix.val[11],
		affine.matrix.val[12], affine.matrix.val[13], affine.matrix.val[14], affine.matrix.val[15]
	);
	matrix.optimize();
	return matrix;
}

cv::Mat eulerAnglesToRotationMatrix(const QVector3D& rotation) {
	if (std::isnan(rotation[0]) && std::isnan(rotation[1]) && std::isnan(rotation[2])) {
		// nan nan nan, Batman!
		return cv::Mat();
	}
	auto matrix(QQuaternion::fromEulerAngles(rotation).toRotationMatrix());
	cv::Matx33f matx(
	    matrix(0, 0), matrix(0, 1), matrix(0, 2),
	    -matrix(1, 0), -matrix(1, 1), -matrix(1, 2),
	    -matrix(2, 0), -matrix(2, 1), -matrix(2, 2)
	);
	return cv::Mat(matx);
}

void rotatePose(QMatrix4x4& pose, const QQuaternion& quaternion) {
	pose.rotate(quaternion);
	auto translation(quaternion.rotatedVector(pose.column(3).toVector3D()));
	pose.setColumn(3, QVector4D(translation, 1));
}




struct CalibrationPose {
	cv::Point2f center;
	std::vector<float> angles;
	CalibrationPose(const std::vector<cv::Point2f>& points) {
		auto prev(points.back());
		for (auto it(points.begin()); it != points.end(); ++it) {
			auto& point(*it);
			center += point;

			auto diff(point - prev);
			angles.push_back(std::atan2(diff.y, diff.x));
			prev = point;
		}
		center /= float(points.size());
		std::sort(angles.begin(), angles.end());
	}
	bool operator==(const CalibrationPose& that) {
		auto center(cv::norm(this->center - that.center));
		if (center > 0.2) {
			return false;
		}

		auto angles(cv::norm(this->angles, that.angles));
		if (angles > 0.2) {
			return false;
		}

		return true;
	}
	bool operator!=(const CalibrationPose& that) {
		return !(*this == that);
	}
};

struct CalibrationExpect {
	bool right;
	QMatrix4x4 transform;
	CalibrationPose pose;
	CalibrationExpect(bool right, QPointF a, QPointF b, QPointF c, QPointF d)
	    : right(right)
	    , transform(squareToQuad(QVector<QPointF> {a, b, c, d}))
	    , pose(std::vector<cv::Point2f> { p(a), p(b), p(c), p(d) }) {
	}
private:
	static QTransform squareToQuad(QPolygonF quad) {
		QTransform transform;
		if (!QTransform::squareToQuad(quad, transform)) {
			qFatal("argl!");
		}
		return transform;
	}
	static cv::Point2f p(const QPointF& point) {
		return cv::Point2f(point.x(), point.y());
	}
};

static const std::vector<CalibrationExpect> calibrationExpects {
	{false, {0.8, 0.8}, {0.2, 0.8}, {0.2, 0.2}, {0.8, 0.2}},
	{true, {0.8, 0.8}, {0.2, 0.8}, {0.2, 0.2}, {0.8, 0.2}},
	{false, {0.8, 0.9}, {0.2, 0.8}, {0.2, 0.2}, {0.8, 0.1}},
	{true, {0.8, 0.9}, {0.2, 0.8}, {0.2, 0.2}, {0.8, 0.1}},
	{false, {0.9, 0.8}, {0.1, 0.8}, {0.2, 0.2}, {0.8, 0.2}},
	{true, {0.9, 0.8}, {0.1, 0.8}, {0.2, 0.2}, {0.8, 0.2}},
};




struct Input {
	QVector3D rotation;
	cv::Mat image;
};

struct OutputRobot {
	QVector3D rotation;
	QMatrix4x4 pose;
};

struct OutputLandmarks {
	QVector3D rotation;
	QList<QMatrix4x4> poses;
};




class VisionVideoFilterRunnable : public QVideoFilterRunnable {
public:
	explicit VisionVideoFilterRunnable(VisionVideoFilter* filter, cv::FileStorage& calibration, cv::FileStorage& geomHashing, cv::FileStorage& robotModel, std::vector<cv::FileStorage>& landmarks);
	~VisionVideoFilterRunnable();
	QVideoFrame run(QVideoFrame* input, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags);
private:
	VisionVideoFilter* filter;
	thymio_tracker::ThymioTracker tracker;

	QThread threadRobot;
	QThread threadLandmarks;
	Runnable runnableRobot;
	Runnable runnableLandmarks;
	TripleBuffer<Input> inputRobot;
	TripleBuffer<Input> inputLandmarks;
	TripleBuffer<OutputRobot> outputRobot;
	TripleBuffer<OutputLandmarks> outputLandmarks;
	bool trackRobot();
	bool trackLandmarks();
	void trackedRobot();
	void trackedLandmarks();
	void updateCalibration(const cv::Mat &input);
	void updateCalibrationExpect();

	size_t calibrationState;

	QOpenGLExtraFunctions* gl;
	QOpenGLShaderProgram program;
	GLint imageLocation;
	GLuint framebuffer;
	GLuint renderbuffer;

	cv::Mat temp1;
	cv::Mat temp2;
};

VisionVideoFilterRunnable::VisionVideoFilterRunnable(VisionVideoFilter* f, cv::FileStorage& calibration, cv::FileStorage& geomHashing, cv::FileStorage& robotModel, std::vector<cv::FileStorage>& landmarks)
		: filter(f)
		, tracker(calibration, geomHashing, robotModel, landmarks)
		, runnableRobot(threadRobot, std::bind(&VisionVideoFilterRunnable::trackRobot, this))
		, runnableLandmarks(threadLandmarks, std::bind(&VisionVideoFilterRunnable::trackLandmarks, this))
		, calibrationState(0)
		, gl(nullptr) {
	updateCalibrationExpect();

	QObject::connect(&runnableRobot, &Runnable::ran, filter, [this]() {
		outputRobot.readSwap();
		trackedRobot();
	}, Qt::QueuedConnection);

	QObject::connect(&runnableLandmarks, &Runnable::ran, filter, [this]() {
		outputLandmarks.readSwap();
		trackedLandmarks();
	}, Qt::QueuedConnection);

	QObject::connect(&filter->sensor, &QSensor::readingChanged, filter, [this]() {
		trackedRobot();
		trackedLandmarks();
	});

	threadRobot.start();
	threadLandmarks.start();
}

VisionVideoFilterRunnable::~VisionVideoFilterRunnable() {
	threadRobot.quit();
	threadLandmarks.quit();
	threadRobot.wait();
	threadLandmarks.wait();
}

QVideoFrame VisionVideoFilterRunnable::run(QVideoFrame* inputFrame, const QVideoSurfaceFormat& surfaceFormat, RunFlags flags) {
	Q_UNUSED(surfaceFormat);
	Q_UNUSED(flags);

	auto& input(inputRobot.writeBuffer());

	auto inputReading(filter->sensor.reading());
	if (inputReading != nullptr) {
		input.rotation = QVector3D(inputReading->x(), inputReading->y(), inputReading->z());
	} else {
		input.rotation = QVector3D(NaN, NaN, NaN);
	}
	//qWarning() << outputReading.val[0] << outputReading.val[1] << outputReading.val[2];

	auto size(inputFrame->size());
	auto height(size.height());
	auto width(size.width());
	auto outputWidth(outputHeight * width / height);

	if (inputFrame->handleType() == QAbstractVideoBuffer::HandleType::GLTextureHandle) {

		if (gl == nullptr) {
			auto context(QOpenGLContext::currentContext());
			gl = context->extraFunctions();

			auto version(context->isOpenGLES() ? "#version 300 es\n" : "#version 130\n");

			QString vertex(version);
			vertex += R"(
				out vec2 coord;
				void main(void) {
					int id = gl_VertexID;
					coord = vec2((id << 1) & 2, id & 2);
					gl_Position = vec4(coord * 2.0 - 1.0, 0.0, 1.0);
				}
			)";

			QString fragment(version);
			fragment += R"(
				in lowp vec2 coord;
				uniform sampler2D image;
				const lowp vec3 luma = vec3(0.2126, 0.7152, 0.0722);
				out lowp float fragment;
				void main(void) {
					lowp vec3 color = texture(image, coord).rgb;
					fragment = dot(color, luma);
				}
			)";

			program.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex);
			program.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment);
			program.link();
			imageLocation = program.uniformLocation("image");

			gl->glGenRenderbuffers(1, &renderbuffer);
			gl->glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
			gl->glRenderbufferStorage(GL_RENDERBUFFER, QOpenGLTexture::R8_UNorm, outputWidth, outputHeight);
			gl->glBindRenderbuffer(GL_RENDERBUFFER, 0);

			gl->glGenFramebuffers(1, &framebuffer);
			gl->glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
			gl->glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, renderbuffer);
			gl->glBindFramebuffer(GL_FRAMEBUFFER, 0);
		}

		gl->glActiveTexture(GL_TEXTURE0);
		gl->glBindTexture(QOpenGLTexture::Target2D, inputFrame->handle().toUInt());
		gl->glGenerateMipmap(QOpenGLTexture::Target2D);
		gl->glTexParameteri(QOpenGLTexture::Target2D, GL_TEXTURE_MIN_FILTER, QOpenGLTexture::LinearMipMapLinear);

		program.bind();
		program.setUniformValue(imageLocation, 0);
		program.enableAttributeArray(0);
		gl->glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
		gl->glViewport(0, 0, outputWidth, outputHeight);
		gl->glDisable(GL_BLEND);
		gl->glDrawArrays(GL_TRIANGLES, 0, 3);

		input.image.create(outputHeight, outputWidth, CV_8UC1);
		gl->glPixelStorei(GL_PACK_ALIGNMENT, 1);
		gl->glReadPixels(0, 0, outputWidth, outputHeight, QOpenGLTexture::Red, QOpenGLTexture::UInt8, input.image.data);

	} else {

		inputFrame->map(QAbstractVideoBuffer::ReadOnly);

		auto pixelFormat(inputFrame->pixelFormat());
		auto inputType(getCvType(pixelFormat));
		auto cvtCode(getCvtCode(pixelFormat));

		auto bits(inputFrame->bits());
		auto bytesPerLine(inputFrame->bytesPerLine());

		//qWarning() << inputType << bits << bytesPerLine;
		auto inputMat(cv::Mat(height, width, inputType, bits, bytesPerLine));

		const auto outputSize(cv::Size(outputWidth, outputHeight));
		const auto outputType(CV_8UC1);

		auto resize(inputMat.size() != outputSize);
		auto convert(cvtCode != cv::COLOR_COLORCVT_MAX);
		auto flip(surfaceFormat.scanLineDirection() == QVideoSurfaceFormat::BottomToTop || QSysInfo::productType() == "android");

		if (resize && convert && flip) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::resize(temp1, temp2, outputSize);
			cv::flip(temp2, input.image, 0);
		} else if (resize && convert) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::resize(temp1, input.image, outputSize);
		} else if (resize && flip) {
			cv::resize(inputMat, temp1, outputSize);
			cv::flip(temp1, input.image, 0);
		} else if (convert && flip) {
			cv::cvtColor(inputMat, temp1, cvtCode, outputType);
			cv::flip(temp1, input.image, 0);
		} else if (resize) {
			cv::resize(inputMat, input.image, outputSize);
		} else if (convert) {
			cv::cvtColor(inputMat, input.image, cvtCode, outputType);
		} else if (flip) {
			cv::flip(inputMat, input.image, 0);
		} else {
			inputMat.copyTo(input.image);
		}

		inputFrame->unmap();

	}

#ifdef THYMIO_AR_IMWRITE
	static bool first = true;
	if (first) {
		qWarning()
				<< input.image.data[0x00] << input.image.data[0x01] << input.image.data[0x02] << input.image.data[0x03]
				<< input.image.data[0x04] << input.image.data[0x05] << input.image.data[0x06] << input.image.data[0x07]
				<< input.image.data[0x08] << input.image.data[0x09] << input.image.data[0x0A] << input.image.data[0x0B]
				<< input.image.data[0x0C] << input.image.data[0x0D] << input.image.data[0x0E] << input.image.data[0x0F]
				<< input.image.data[0x10] << input.image.data[0x11] << input.image.data[0x12] << input.image.data[0x13]
				<< input.image.data[0x14] << input.image.data[0x15] << input.image.data[0x16] << input.image.data[0x17]
				<< input.image.data[0x18] << input.image.data[0x19] << input.image.data[0x1A] << input.image.data[0x1B]
				<< input.image.data[0x1C] << input.image.data[0x1D] << input.image.data[0x1E] << input.image.data[0x1F];
		cv::imwrite(QSysInfo::productType() == "android" ? "/storage/emulated/0/DCIM/100ANDRO/toto.png" : "toto.png", input.image);
	}
	first = false;
#endif

	inputLandmarks.writeBuffer() = input;

	if (!inputRobot.writeSwap()) {
		runnableRobot.invoke();
	}
	if (!inputLandmarks.writeSwap()) {
		runnableLandmarks.invoke();
	}

/*/
	return *inputFrame;
/*/
	QVideoFrame frame(input.image.size().area()*3/2, QSize(input.image.cols, input.image.rows), input.image.step, QVideoFrame::Format_YUV420P);
	frame.setStartTime(inputFrame->startTime());
	frame.setEndTime(inputFrame->endTime());
	frame.map(QAbstractVideoBuffer::ReadWrite);
	std::memcpy(frame.bits(), input.image.data, input.image.size().area());
	std::memset(frame.bits() + input.image.size().area(), 127, input.image.size().area() / 2);
	frame.unmap();
	return frame;
/**/
}

bool VisionVideoFilterRunnable::trackRobot() {
	inputRobot.readSwap();
	const auto& input(inputRobot.readBuffer());

	auto rotation(eulerAnglesToRotationMatrix(input.rotation));
	tracker.updateRobot(input.image, rotation.empty() ? nullptr : &rotation);

	auto& output(outputRobot.writeBuffer());
	output.rotation = input.rotation;

	const auto& detection(tracker.getDetectionInfo());
	output.pose = cvAffine3dToQMatrix4x4(detection.mRobotDetection.isFound(), detection.mRobotDetection.getPose());

	return !outputRobot.writeSwap();
}

bool VisionVideoFilterRunnable::trackLandmarks() {
	inputLandmarks.readSwap();
	const auto& input(inputLandmarks.readBuffer());

	auto rotation(eulerAnglesToRotationMatrix(input.rotation));
	tracker.updateLandmarks(input.image, rotation.empty() ? nullptr : &rotation);
//	qWarning() << tracker.getTimer().getFps();

	auto& output(outputLandmarks.writeBuffer());
	output.rotation = input.rotation;

	const auto& detection(tracker.getDetectionInfo());
	output.poses.clear();
	output.poses.reserve(detection.landmarkDetections.size());
	for (const auto& landmark : detection.landmarkDetections) {
		output.poses.push_back(cvAffine3dToQMatrix4x4(landmark.isFound(), landmark.getPose()));
	}

	if (filter->calibrationRunning) {
		updateCalibration(input.image);
	}

	return !outputLandmarks.writeSwap();
}

void VisionVideoFilterRunnable::trackedRobot() {
	const auto& output(outputRobot.readBuffer());

	filter->robotPose = output.pose;

	auto reading(filter->sensor.reading());
	if (reading != nullptr) {
		auto rotation(QVector3D(reading->x(), reading->y(), reading->z()));
		auto diff(output.rotation - rotation);
		auto quaternion(QQuaternion::fromEulerAngles(diff));

		rotatePose(filter->robotPose, quaternion);
	}

	emit filter->updatedRobot();
}

void VisionVideoFilterRunnable::trackedLandmarks() {
	const auto& output(outputLandmarks.readBuffer());

	filter->landmarkPoses.clear();

	auto reading(filter->sensor.reading());
	if (reading != nullptr) {
		auto rotation(QVector3D(reading->x(), reading->y(), reading->z()));
		auto diff(output.rotation - rotation);
		auto quaternion(QQuaternion::fromEulerAngles(diff));

		for (auto pose : output.poses) {
			rotatePose(pose, quaternion);
			filter->landmarkPoses.append(QVariant::fromValue(pose));
		}
	} else {
		for (auto& pose : output.poses) {
			filter->landmarkPoses.append(QVariant::fromValue(pose));
		}
	}

	emit filter->updatedLandmarks();
}

void VisionVideoFilterRunnable::updateCalibration(const cv::Mat& input) {
	const auto& detections(tracker.getDetectionInfo().landmarkDetections);
	if (detections.empty()) {
		// no landmark
		return;
	}

	const auto& detection(detections.front());
	if (!detection.isFound()) {
		// invisible landmark
		return;
	}

	const auto& landmark(tracker.getLandmarks().front());

	std::vector<cv::Point2f> landmarkCorners;
	cv::perspectiveTransform(landmark.getCorners(), landmarkCorners, detection.getHomography());
	CalibrationPose landmarkPose(landmarkCorners);

	auto size(input.size());
	if (filter->calibrationRight) {
		landmarkPose.center.x += size.height - size.width;
	}
	landmarkPose.center /= float(size.height);

	const auto& calibrationPose(calibrationExpects[calibrationState].pose);
	if (landmarkPose != calibrationPose) {
		// wrong pose
		return;
	}

	if (!tracker.updateCalibration()) {
		calibrationState += 1;
		calibrationState %= calibrationExpects.size();
		filter->calibrationProgress = tracker.getCalibrationInfo().getProgress();
		filter->calibrationDone = false;
	} else {
		calibrationState = 0;
		filter->calibrationRunning = false;
		filter->calibrationProgress = 1.0;
		filter->calibrationDone = true;

		cv::FileStorage storage("calibration.xml", cv::FileStorage::WRITE | cv::FileStorage::MEMORY);
		tracker.writeCalibration(storage);
		auto calibration(QString::fromStdString(storage.releaseAndGetString()));

		QSettings settings;
		settings.setValue("thymio-ar/calibration.xml", calibration);
	}

	updateCalibrationExpect();
}

void VisionVideoFilterRunnable::updateCalibrationExpect() {
	auto& calibrationExpect(calibrationExpects[calibrationState]);
	filter->calibrationRight = calibrationExpect.right;
	filter->calibrationTransform = calibrationExpect.transform;
	emit filter->updatedCalibration();
}




VisionVideoFilter::VisionVideoFilter(QObject* parent)
		: QAbstractVideoFilter(parent) {
	sensor.start();
}

static std::string readFile(QString path) {
	QFile file(path);
	if (!file.open(QFile::ReadOnly)) {
		qFatal("Cannot open file %s", path.toLocal8Bit().constData());
	}
	return file.readAll().toStdString();
}

static std::string readSettingFile(QString path) {
	static QSettings settings;
	auto variant(settings.value(path));
	if (variant.isValid()) {
		return variant.value<QString>().toStdString();
	}

	return readFile(":/" + path);
}

QVideoFilterRunnable* VisionVideoFilter::createFilterRunnable() {
	cv::FileStorage calibration(readSettingFile("thymio-ar/calibration.xml"), cv::FileStorage::READ | cv::FileStorage::MEMORY);
	cv::FileStorage geomHashing(readFile(":/thymio-ar/geomHashing.xml"), cv::FileStorage::READ | cv::FileStorage::MEMORY);
	cv::FileStorage robotModel(readFile(":/thymio-ar/robotModel.xml"), cv::FileStorage::READ | cv::FileStorage::MEMORY);
	std::vector<cv::FileStorage> landmarks;
	for (auto landmarkFileName : this->landmarkFileNames) {
		landmarks.push_back(cv::FileStorage(readFile(landmarkFileName), cv::FileStorage::READ | cv::FileStorage::MEMORY));
	}
	return new VisionVideoFilterRunnable(this, calibration, geomHashing, robotModel, landmarks);
}

const QVariantList& VisionVideoFilter::getLandmarkPoses() {
	while (landmarkPoses.size() < landmarkFileNames.size()) {
		landmarkPoses.append(QMatrix4x4());
	}
	if (landmarkPoses.size() > landmarkFileNames.size()) {
		landmarkPoses.erase(landmarkPoses.begin() + landmarkFileNames.size(), landmarkPoses.end());
	}
	return landmarkPoses;
}

// This file declares Q_OBJECT classes, so we need to
#include "vision-video-filter.moc"
