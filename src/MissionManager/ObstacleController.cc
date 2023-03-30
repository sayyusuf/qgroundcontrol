/****************************************************************************
 *
 * (c) 2009-2020 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/


/// @file
///     @author Don Gagne <don@thegagnes.com>

#include "ObstacleController.h"
#include "Vehicle.h"
#include "FirmwarePlugin.h"
#include "MAVLinkProtocol.h"
#include "QGCApplication.h"
#include "ParameterManager.h"
#include "JsonHelper.h"
#include "QGCQGeoCoordinate.h"
#include "AppSettings.h"
#include "PlanMasterController.h"
#include "SettingsManager.h"
#include "AppSettings.h"

#include <QJsonDocument>
#include <QJsonArray>

QGC_LOGGING_CATEGORY(ObstacleControllerLog, "ObstacleControllerLog")

QMap<QString, FactMetaData*> ObstacleController::_metaDataMap;

const char* ObstacleController::_jsonFileTypeValue =        "Obstacle";
const char* ObstacleController::_jsonBreachReturnKey =      "breachReturn";
const char* ObstacleController::_jsonPolygonsKey =          "polygons";
const char* ObstacleController::_jsonCirclesKey =           "circles";

const char* ObstacleController::_breachReturnAltitudeFactName = "Altitude";

const char* ObstacleController::_px4ParamCircularFence =    "GF_MAX_HOR_DIST";

ObstacleController::ObstacleController(PlanMasterController* masterController, QObject* parent)
    : PlanElementController         (masterController, parent)
    , _managerVehicle               (masterController->managerVehicle())
    , _obstacleManager              (masterController->managerVehicle()->obstacleManager())
    , _breachReturnAltitudeFact     (0, _breachReturnAltitudeFactName, FactMetaData::valueTypeDouble)
    , _breachReturnDefaultAltitude  (qgcApp()->toolbox()->settingsManager()->appSettings()->defaultMissionItemAltitude()->rawValue().toDouble())
{
    if (_metaDataMap.isEmpty()) {
        _metaDataMap = FactMetaData::createMapFromJsonFile(QStringLiteral(":/json/BreachReturn.FactMetaData.json"), nullptr /* metaDataParent */);
    }

    _breachReturnAltitudeFact.setMetaData(_metaDataMap[_breachReturnAltitudeFactName]);
    _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);

    connect(&_polygons, &QmlObjectListModel::countChanged, this, &ObstacleController::_updateContainsItems);
    connect(&_circles,  &QmlObjectListModel::countChanged, this, &ObstacleController::_updateContainsItems);

    connect(this,                       &ObstacleController::breachReturnPointChanged,  this, &ObstacleController::_setDirty);
    connect(&_breachReturnAltitudeFact, &Fact::rawValueChanged,                         this, &ObstacleController::_setDirty);
    connect(&_polygons,                 &QmlObjectListModel::dirtyChanged,              this, &ObstacleController::_setDirty);
    connect(&_circles,                  &QmlObjectListModel::dirtyChanged,              this, &ObstacleController::_setDirty);
}

ObstacleController::~ObstacleController()
{

}

void ObstacleController::start(bool flyView)
{
    qCDebug(ObstacleControllerLog) << "start flyView" << flyView;

    _managerVehicleChanged(_masterController->managerVehicle());
    connect(_masterController, &PlanMasterController::managerVehicleChanged, this, &ObstacleController::_managerVehicleChanged);

    PlanElementController::start(flyView);
    _init();
}

void ObstacleController::_init(void)
{

}

void ObstacleController::setBreachReturnPoint(const QGeoCoordinate& breachReturnPoint)
{
    if (_breachReturnPoint != breachReturnPoint) {
        _breachReturnPoint = breachReturnPoint;
        setDirty(true);
        emit breachReturnPointChanged(breachReturnPoint);
    }
}

void ObstacleController::_managerVehicleChanged(Vehicle* managerVehicle)
{
    if (_managerVehicle) {
        _obstacleManager->disconnect(this);
        _managerVehicle->disconnect(this);
        _managerVehicle->parameterManager()->disconnect(this);
        _managerVehicle = nullptr;
        _obstacleManager = nullptr;
    }

    _managerVehicle = managerVehicle;
    if (!_managerVehicle) {
        qWarning() << "ObstacleController::managerVehicleChanged managerVehicle=nullptr";
        return;
    }

    _obstacleManager = _managerVehicle->obstacleManager();
    connect(_obstacleManager, &ObstacleManager::loadComplete,                   this, &ObstacleController::_managerLoadComplete);
    connect(_obstacleManager, &ObstacleManager::sendComplete,                   this, &ObstacleController::_managerSendComplete);
    connect(_obstacleManager, &ObstacleManager::removeAllComplete,              this, &ObstacleController::_managerRemoveAllComplete);
    connect(_obstacleManager, &ObstacleManager::inProgressChanged,              this, &ObstacleController::syncInProgressChanged);

    //-- ObstacleController::supported() tests both the capability bit AND the protocol version.
    connect(_managerVehicle,  &Vehicle::capabilityBitsChanged,                  this, &ObstacleController::supportedChanged);
    connect(_managerVehicle,  &Vehicle::requestProtocolVersion,                 this, &ObstacleController::supportedChanged);

    connect(_managerVehicle->parameterManager(), &ParameterManager::parametersReadyChanged, this, &ObstacleController::_parametersReady);
    _parametersReady();

    emit supportedChanged(supported());
}

bool ObstacleController::load(const QJsonObject& json, QString& errorString)
{
    removeAll();

    errorString.clear();

    if (!json.contains(JsonHelper::jsonVersionKey) ||
            (json.contains(JsonHelper::jsonVersionKey) && json[JsonHelper::jsonVersionKey].toInt() == 1)) {
        // We just ignore old version 1 or prior data
        return true;
    }

    QList<JsonHelper::KeyValidateInfo> keyInfoList = {
        { JsonHelper::jsonVersionKey,   QJsonValue::Double, true },
        { _jsonCirclesKey,              QJsonValue::Array,  true },
        { _jsonPolygonsKey,             QJsonValue::Array,  true },
        { _jsonBreachReturnKey,         QJsonValue::Array,  false },
    };
    if (!JsonHelper::validateKeys(json, keyInfoList, errorString)) {
        return false;
    }

    if (json[JsonHelper::jsonVersionKey].toInt() != _jsonCurrentVersion) {
        errorString = tr("Obstacle supports version %1").arg(_jsonCurrentVersion);
        return false;
    }

    QJsonArray jsonPolygonArray = json[_jsonPolygonsKey].toArray();
    for (const QJsonValue jsonPolygonValue: jsonPolygonArray) {
        if (jsonPolygonValue.type() != QJsonValue::Object) {
            errorString = tr("Obstacle polygon not stored as object");
            return false;
        }

        QGCFencePolygon* fencePolygon = new QGCFencePolygon(false /* inclusion */, this /* parent */);
        if (!fencePolygon->loadFromJson(jsonPolygonValue.toObject(), true /* required */, errorString)) {
            return false;
        }
        _polygons.append(fencePolygon);
    }

    QJsonArray jsonCircleArray = json[_jsonCirclesKey].toArray();
    for (const QJsonValue jsonCircleValue: jsonCircleArray) {
        if (jsonCircleValue.type() != QJsonValue::Object) {
            errorString = tr("Obstacle circle not stored as object");
            return false;
        }

        QGCFenceCircle* fenceCircle = new QGCFenceCircle(this /* parent */);
        if (!fenceCircle->loadFromJson(jsonCircleValue.toObject(), errorString)) {
            return false;
        }
        _circles.append(fenceCircle);
    }

    if (json.contains(_jsonBreachReturnKey)) {
        if (!JsonHelper::loadGeoCoordinate(json[_jsonBreachReturnKey], true /* altitudeRequred */, _breachReturnPoint, errorString)) {
            return false;
        }
        _breachReturnAltitudeFact.setRawValue(_breachReturnPoint.altitude());
    } else {
        _breachReturnPoint = QGeoCoordinate();
        _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);
    }
    emit breachReturnPointChanged(_breachReturnPoint);

    setDirty(false);

    return true;
}

void ObstacleController::save(QJsonObject& json)
{
    json[JsonHelper::jsonVersionKey] = _jsonCurrentVersion;

    QJsonArray jsonPolygonArray;
    for (int i=0; i<_polygons.count(); i++) {
        QJsonObject jsonPolygon;
        QGCFencePolygon* fencePolygon = _polygons.value<QGCFencePolygon*>(i);
        fencePolygon->saveToJson(jsonPolygon);
        jsonPolygonArray.append(jsonPolygon);
    }
    json[_jsonPolygonsKey] = jsonPolygonArray;

    QJsonArray jsonCircleArray;
    for (int i=0; i<_circles.count(); i++) {
        QJsonObject jsonCircle;
        QGCFenceCircle* fenceCircle = _circles.value<QGCFenceCircle*>(i);
        fenceCircle->saveToJson(jsonCircle);
        jsonCircleArray.append(jsonCircle);
    }
    json[_jsonCirclesKey] = jsonCircleArray;

    if (_breachReturnPoint.isValid()) {
        QJsonValue jsonCoordinate;

        _breachReturnPoint.setAltitude(_breachReturnAltitudeFact.rawValue().toDouble());
        JsonHelper::saveGeoCoordinate(_breachReturnPoint, true /* writeAltitude */, jsonCoordinate);
        json[_jsonBreachReturnKey] = jsonCoordinate;
    }
}

void ObstacleController::removeAll(void)
{    
    setBreachReturnPoint(QGeoCoordinate());
    _polygons.clearAndDeleteContents();
    _circles.clearAndDeleteContents();
}

void ObstacleController::removeAllFromVehicle(void)
{
    if (_masterController->offline()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::removeAllFromVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::removeAllFromVehicle called while syncInProgress";
    } else {
        _obstacleManager->removeAll();
    }
}

void ObstacleController::loadFromVehicle(void)
{
    if (_masterController->offline()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::loadFromVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::loadFromVehicle called while syncInProgress";
    } else {
        _itemsRequested = true;
        _obstacleManager->loadFromVehicle();
    }
}

void ObstacleController::sendToVehicle(void)
{
    if (_masterController->offline()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::sendToVehicle called while offline";
    } else if (syncInProgress()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::sendToVehicle called while syncInProgress";
    } else {
        qCDebug(ObstacleControllerLog) << "ObstacleController::sendToVehicle";
        _obstacleManager->sendToVehicle(_breachReturnPoint, _polygons, _circles);
        setDirty(false);
    }
}

bool ObstacleController::syncInProgress(void) const
{
    return _obstacleManager->inProgress();
}

bool ObstacleController::dirty(void) const
{
    return _dirty;
}


void ObstacleController::setDirty(bool dirty)
{
    if (dirty != _dirty) {
        _dirty = dirty;
        if (!dirty) {
            for (int i=0; i<_polygons.count(); i++) {
                QGCFencePolygon* polygon = _polygons.value<QGCFencePolygon*>(i);
                polygon->setDirty(false);
            }
            for (int i=0; i<_circles.count(); i++) {
                QGCFenceCircle* circle = _circles.value<QGCFenceCircle*>(i);
                circle->setDirty(false);
            }
        }
        emit dirtyChanged(dirty);
    }
}

void ObstacleController::_polygonDirtyChanged(bool dirty)
{
    if (dirty) {
        setDirty(true);
    }
}

void ObstacleController::_setDirty(void)
{
    setDirty(true);
}

void ObstacleController::_setFenceFromManager(const QList<QGCFencePolygon>& polygons,
                                              const QList<QGCFenceCircle>&  circles)
{
    _polygons.clearAndDeleteContents();
    _circles.clearAndDeleteContents();

    for (int i=0; i<polygons.count(); i++) {
        _polygons.append(new QGCFencePolygon(polygons[i], this));
    }

    for (int i=0; i<circles.count(); i++) {
        _circles.append(new QGCFenceCircle(circles[i], this));
    }

    setDirty(false);
}

void ObstacleController::_setReturnPointFromManager(QGeoCoordinate breachReturnPoint)
{
    _breachReturnPoint = breachReturnPoint;
    emit breachReturnPointChanged(_breachReturnPoint);
    if (_breachReturnPoint.isValid()) {
        _breachReturnAltitudeFact.setRawValue(_breachReturnPoint.altitude());
    } else {
        _breachReturnAltitudeFact.setRawValue(_breachReturnDefaultAltitude);
    }
}

void ObstacleController::_managerLoadComplete(void)
{
    // Fly view always reloads on _loadComplete
    // Plan view only reloads if:
    //  - Load was specifically requested
    //  - There is no current Plan
    if (_flyView || _itemsRequested || isEmpty()) {
        _setReturnPointFromManager(_obstacleManager->breachReturnPoint());
        _setFenceFromManager(_obstacleManager->polygons(), _obstacleManager->circles());
        setDirty(false);
        emit loadComplete();
    }
    _itemsRequested = false;
}

void ObstacleController::_managerSendComplete(bool error)
{
    // Fly view always reloads on manager sendComplete
    if (!error && _flyView) {
        showPlanFromManagerVehicle();
    }
}

void ObstacleController::_managerRemoveAllComplete(bool error)
{
    if (!error) {
        // Remove all from vehicle so we always update
        showPlanFromManagerVehicle();
    }
}

bool ObstacleController::containsItems(void) const
{
    return _polygons.count() > 0 || _circles.count() > 0;
}

void ObstacleController::_updateContainsItems(void)
{
    emit containsItemsChanged(containsItems());
}

bool ObstacleController::showPlanFromManagerVehicle(void)
{
    qCDebug(ObstacleControllerLog) << "showPlanFromManagerVehicle _flyView" << _flyView;
    if (_masterController->offline()) {
        qCWarning(ObstacleControllerLog) << "ObstacleController::showPlanFromManagerVehicle called while offline";
        return true;    // stops further propagation of showPlanFromManagerVehicle due to error
    } else {
        _itemsRequested = true;
        if (!_managerVehicle->initialPlanRequestComplete()) {
            // The vehicle hasn't completed initial load, we can just wait for loadComplete to be signalled automatically
            qCDebug(ObstacleControllerLog) << "showPlanFromManagerVehicle: !initialPlanRequestComplete, wait for signal";
            return true;
        } else if (syncInProgress()) {
            // If the sync is already in progress, _loadComplete will be called automatically when it is done. So no need to do anything.
            qCDebug(ObstacleControllerLog) << "showPlanFromManagerVehicle: syncInProgress wait for signal";
            return true;
        } else {
            // Fake a _loadComplete with the current items
            qCDebug(ObstacleControllerLog) << "showPlanFromManagerVehicle: sync complete simulate signal";
            _itemsRequested = true;
            _managerLoadComplete();
            return false;
        }
    }
}

void ObstacleController::addInclusionPolygon(QGeoCoordinate topLeft, QGeoCoordinate bottomRight)
{
    QGeoCoordinate topRight(topLeft.latitude(), bottomRight.longitude());
    QGeoCoordinate bottomLeft(bottomRight.latitude(), topLeft.longitude());

    double halfWidthMeters = topLeft.distanceTo(topRight) / 2.0;
    double halfHeightMeters = topLeft.distanceTo(bottomLeft) / 2.0;

    QGeoCoordinate centerLeftEdge = topLeft.atDistanceAndAzimuth(halfHeightMeters, 180);
    QGeoCoordinate centerTopEdge = topLeft.atDistanceAndAzimuth(halfWidthMeters, 90);
    QGeoCoordinate center(centerLeftEdge.latitude(), centerTopEdge.longitude());

    // Initial polygon is inset to take 3/4s of viewport with max width/height of 3000 meters
    halfWidthMeters =   qMin(halfWidthMeters * 0.75, 1500.0);
    halfHeightMeters =  qMin(halfHeightMeters * 0.75, 1500.0);

    // Initial polygon has max width and height of 3000 meters
    topLeft =           center.atDistanceAndAzimuth(halfWidthMeters, -90).atDistanceAndAzimuth(halfHeightMeters, 0);
    topRight =          center.atDistanceAndAzimuth(halfWidthMeters, 90).atDistanceAndAzimuth(halfHeightMeters, 0);
    bottomLeft =        center.atDistanceAndAzimuth(halfWidthMeters, -90).atDistanceAndAzimuth(halfHeightMeters, 180);
    bottomRight =       center.atDistanceAndAzimuth(halfWidthMeters, 90).atDistanceAndAzimuth(halfHeightMeters, 180);

    QGCFencePolygon* polygon = new QGCFencePolygon(true /* inclusion */, this);
    polygon->appendVertex(topLeft);
    polygon->appendVertex(topRight);
    polygon->appendVertex(bottomRight);
    polygon->appendVertex(bottomLeft);
    _polygons.append(polygon);

    clearAllInteractive();
    polygon->setInteractive(true);
}

void ObstacleController::addInclusionCircle(QGeoCoordinate topLeft, QGeoCoordinate bottomRight)
{
    QGeoCoordinate topRight(topLeft.latitude(), bottomRight.longitude());
    QGeoCoordinate bottomLeft(bottomRight.latitude(), topLeft.longitude());

    // Initial radius is inset to take 3/4s of viewport and max of 1500 meters
    double halfWidthMeters = topLeft.distanceTo(topRight) / 2.0;
    double halfHeightMeters = topLeft.distanceTo(bottomLeft) / 2.0;
    double radius = qMin(qMin(halfWidthMeters, halfHeightMeters) * 0.75, 1500.0);

    QGeoCoordinate centerLeftEdge = topLeft.atDistanceAndAzimuth(halfHeightMeters, 180);
    QGeoCoordinate centerTopEdge = topLeft.atDistanceAndAzimuth(halfWidthMeters, 90);
    QGeoCoordinate center(centerLeftEdge.latitude(), centerTopEdge.longitude());

    QGCFenceCircle* circle = new QGCFenceCircle(center, radius, true /* inclusion */, this);
    _circles.append(circle);

    clearAllInteractive();
    circle->setInteractive(true);
}

void ObstacleController::deletePolygon(int index)
{
    if (index < 0 || index > _polygons.count() - 1) {
        return;
    }

    QGCFencePolygon* polygon = qobject_cast<QGCFencePolygon*>(_polygons.removeAt(index));
    polygon->deleteLater();
}

void ObstacleController::deleteCircle(int index)
{
    if (index < 0 || index > _circles.count() - 1) {
        return;
    }

    QGCFenceCircle* circle = qobject_cast<QGCFenceCircle*>(_circles.removeAt(index));
    circle->deleteLater();
}

void ObstacleController::clearAllInteractive(void)
{
    for (int i=0; i<_polygons.count(); i++) {
        _polygons.value<QGCFencePolygon*>(i)->setInteractive(false);
    }
    for (int i=0; i<_circles.count(); i++) {
        _circles.value<QGCFenceCircle*>(i)->setInteractive(false);
    }
}

bool ObstacleController::supported(void) const
{
    return (_managerVehicle->capabilityBits() & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE) && (_managerVehicle->maxProtoVersion() >= 200);
}

// Hack for PX4
double ObstacleController::paramCircularFence(void)
{
    if (_managerVehicle->isOfflineEditingVehicle() || !_managerVehicle->parameterManager()->parameterExists(FactSystem::defaultComponentId, _px4ParamCircularFence)) {
        return 0;
    }

    return _managerVehicle->parameterManager()->getParameter(FactSystem::defaultComponentId, _px4ParamCircularFence)->rawValue().toDouble();
}

void ObstacleController::_parametersReady(void)
{
    if (_px4ParamCircularFenceFact) {
        _px4ParamCircularFenceFact->disconnect(this);
        _px4ParamCircularFenceFact = nullptr;
    }

    if (_managerVehicle->isOfflineEditingVehicle() || !_managerVehicle->parameterManager()->parameterExists(FactSystem::defaultComponentId, _px4ParamCircularFence)) {
        emit paramCircularFenceChanged();
        return;
    }

    _px4ParamCircularFenceFact = _managerVehicle->parameterManager()->getParameter(FactSystem::defaultComponentId, _px4ParamCircularFence);
    connect(_px4ParamCircularFenceFact, &Fact::rawValueChanged, this, &ObstacleController::paramCircularFenceChanged);
    emit paramCircularFenceChanged();
}

bool ObstacleController::isEmpty(void) const
{
    return _polygons.count() == 0 && _circles.count() == 0 && !_breachReturnPoint.isValid();

}
